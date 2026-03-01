use std::cell::RefCell;
use std::f32::consts::PI;
use std::marker::PhantomData;

use embedded_hal::delay;
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_idf_hal::cpu::core;
use esp_idf_hal::delay::{Delay, Ets, FreeRtos};
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use anyhow::Result;
use esp_idf_hal::units::Hertz;
use esp_idf_svc::http::status::OK;
use esp_idf_sys::COLL_WEIGHTS_MAX;
//use pwm_pca9685::{Address, Pca9685,Channel};
use pwm_pca9685::*;
use esp_idf_hal::ledc::*;

const TICK_PERIOD_MS: u32 = 1;

// 버튼 비트마스크 정의 (로그 데이터 기반)
const MASK_L1: u8 = 0x04;       // 11111011
const MASK_R1: u8 = 0x08;       // R1 추정치 (0x10이 세모이므로)
const MASK_TRIANGLE: u8 = 0x10; // 11101111 (세모 버튼)

// [설정] 안정성을 위한 상수
const DEADZONE: i16 = 15;
const SENSITIVITY: f32 = 2.0; // 한 번의 루프에서 변할 최대 각도
const MIN_ANGLE: f32 = 0.0;
const MAX_ANGLE: f32 = 180.0;

// [cite: 2026-02-13] 안정성을 위한 상수 설정
const STEPS: usize = 60; // 궤적 분할 수 (안정적인 이동을 위해 설정)
const SERVO_MIN: u16 = 150; // RDS3225 최소 펄스
const SERVO_MAX: u16 = 600; // RDS3225 최대 펄스

pub struct RobotArmController {
    pub master_id: usize,        // 현재 선택된 모터 번호 (0~5)
    pub angles: [f32; 6],        // 각 모터의 현재 각도 저장
    is_locked: bool,         // 안전을 위한 잠금 장치
}

impl RobotArmController {
    fn new() -> Self {
        Self {
            master_id: 0,
            angles: [90.0; 6], // 초기값은 모두 중앙(90도)
            is_locked: false,
        }
    }

    // 모터별 방향 및 가중치 (1번, 4번은 반전 설치됨을 가정)
    fn get_motor_traits(&self) -> (f32, f32) {
        match self.master_id {
            1 => (-1.0, 2.5), // #1 어깨: 반전, 고출력 가중치
            3 => (1.0, 1.8),  // #3 팔꿈치: 정방향
            4 => (-1.0, 1.5), // #4 손목: 반전
            _ => (1.0, 1.2),
        }
    }

    // [조그셔틀/조이스틱 로직] 선택된 마스터 모터만 미세 조정
   fn apply_move(&mut self, joy_delta: f32) {
        let (dir, weight) = self.get_motor_traits();
        // 위로 밀 때(joy_delta > 0) 중력 보상 적용
        let power = if joy_delta > 0.0 { weight } else { 0.7 };
        let final_delta = joy_delta * dir * power;
        
        self.angles[self.master_id] = (self.angles[self.master_id] + final_delta).clamp(0.0, 180.0);
    } 
    
}

// 로봇의 상태 정의
enum RobotState {
    Idle,       // 대기 (중립 위치)
    Scanning,   // 사과 위치 탐색 (좌우 회전)
    Preparing,  // 깎기 시작 지점으로 이동
}

// [cite: 2026-02-02] I2C 제네릭을 추가하여 어떤 I2C 장치든 대응 가능하게 수정
pub struct RobotArm3Axis<I2C> {
    pub current_angles: [f32; 3],
    pub _marker: core::marker::PhantomData<I2C>, // 타입을 저장하기 위한 마커
}

impl<I2C> RobotArm3Axis<I2C>  where I2C: embedded_hal::i2c::I2c  {
    // [cite: 2026-02-02, 2026-02-13] Sine Interpolation 기반 부드러운 3축 이동
    fn move_to_target(&mut self, pca9685: &mut Pca9685<I2C>, target_angles: [f32; 3], delay: &Delay) 
    {
        let start_angles = self.current_angles;

        for step in 0..=STEPS {
            let t = step as f32 / STEPS as f32;
            // [cite: 2026-02-02] Sine Ramp: 가속과 감속을 부드럽게 (Grok's favorite!)
            let s = (1.0 - (t * PI).cos()) / 2.0; 

            for i in 0..3 {
                let interpolated_angle = start_angles[i] + (target_angles[i] - start_angles[i]) * s;
                
                // [cite: 2026-02-06] Safety: 각 관절별 Soft Limit 적용 (0~180도)
                let safe_angle = interpolated_angle.clamp(0.0, 180.0);
                
                // PCA9685 채널 0, 1, 2에 각각 전송 [cite: 2026-01-29, 2026-02-23]
                let pulse = self.angle_to_pulse(safe_angle);
                let channels = [Channel::C0, Channel::C1, Channel::C2];
                //let target_channel = Channel::from(i as u8) ;
                pca9685.set_channel_on_off(channels[i],  0, pulse).unwrap(); //set_pwm(i as u8, 0, pulse).unwrap();
            }
            delay.delay_ms(20u32); // [cite: 2026-02-13] 50Hz 주기에 맞춘 안정적인 딜레이
        }
        self.current_angles = target_angles;
    }

    fn angle_to_pulse(&self, angle: f32) -> u16 {
        (SERVO_MIN as f32 + (angle / 180.0) * (SERVO_MAX - SERVO_MIN) as f32) as u16
    }
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    let peripherals = Peripherals::take()?;

   println!("🚀 [테스트] I2C를 제외하고 PS2 컨트롤러만 시작합니다...");

   // LEDC 설정 (50Hz)
    let timer_config = config::TimerConfig::new()
        .frequency(Hertz(50).into())
        .resolution(Resolution::Bits14);
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config)?;

    let mut motor_0 = LedcDriver::new(
        peripherals.ledc.channel0,
        &timer,
        peripherals.pins.gpio15, // ESP32-C6 핀맵 확인 후 수정
    )?;

    let mut current_state = RobotState::Idle;
    let mut current_duty = 1229; // Neutral

    println!("자동 제어 시스템 시작...");

    // --- I2C 및 PCA9685 로직 일시 중단 ---
    let config = I2cConfig::new().baudrate(Hertz(100_000));
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        &config,
    )?;

    // 1. I2C 드라이버를 RefCell로 감쌉니다.
    let i2c_ref_cell = RefCell::new(i2c);

    // 2. PCA9685용 가상 I2C 핸들을 만듭니다. (소유권 문제 해결)
    let pwm_i2c =  RefCellDevice::new(&i2c_ref_cell);// i2c_bus.acquire_i2c();

    let mut pwm = Pca9685::new(pwm_i2c, Address::from(0x60))
     .map_err(|_| anyhow::anyhow!("PCA9685 초기화 실패"))?;
    
    pwm.set_prescale(121).unwrap();
    pwm.enable().unwrap();
    println!("✅ 모터 드라이버(PCA9685) 연결 성공!");
    //--------------------------------------- 

    // 테스트할 채널 목록 (0번부터 5번까지)
   let channels = [
        //(0, "베이스", Channel::C0),
        (1, "어깨",   Channel::C1),
        (2, "팔꿈치", Channel::C2),
        //(3, "손목/칼날", Channel::C3),
    ];

    // 1. 루프 시작 전 초기 위치 설정 (위쪽으로 접힌 자세 예시: 400)
    let mut current_pos_shoulder = 500u16;

    // ----------- Motor #1 adjust start -------------------------------------------//
    // 위쪽(450)으로 먼저 움직여서 공간 확보
    //let target_upper = 450; 
    let target_upper = 550; 

    

   
    
    // 다시 중간(350)으로 복귀
    //move_arm_smooth(&mut pwm, Channel::C1, &mut current_pos_shoulder, 350);
    //println!("#1 다시 중간(300)으로 복귀");
     
    //--------- #1 모터 adjust end -------------------------------------------------//

    // 초기 위치 저장용 변수
    let mut current_positions = [300u16; 4]; // 0~3번 관절 초기값
    
    // 1. 모든 모터의 '현재 위치'를 기억할 변수들을 루프 밖에서 초기화합니다.
    // 사진 속의 적당한 위치를 400(어깨), 350(팔꿈치) 정도로 가정합니다.
    let mut shoulder_pos = 400u16; 
    //let mut elbow_pos = 350u16;
    let mut elbow_pos = 300u16;
    let mut base_pos = 300u16;
    let mut wrist_pos = 300u16;

    println!("=== 관절로봇 테스트 시작 ===");

     //println!("🔔 2번 팔꿈치 모터 - 180...");
    //                move_arm_smooth(&mut pwm, Channel::C2, &mut elbow_pos, 180);
                    //move_arm_smooth(&mut pwm, *channel, &mut elbow_pos, 100);
                    //move_arm_smooth(&mut pwm, *channel, &mut elbow_pos, 100);
    
    //println!("🔔 1번 어깨 모터 (위쪽 Safe Zone)");
                    // 사진의 위치(400)에서 위아래로 살짝만 움직여 부하 최소화
     //               move_arm_smooth(&mut pwm, Channel::C1, &mut shoulder_pos, 360); // 위로 더 들기
     //               FreeRtos::delay_ms(1200);
     /*move_arm_smooth(&mut pwm, Channel::C1, &mut current_pos_shoulder, target_upper);
    println!("#1 위쪽(550)으로 먼저 움직여서 공간 확보");
    FreeRtos::delay_ms(1000);
    */
    //let mut currents = [shoulder_pos, elbow_pos, wrist_pos, 300u16];

    // 1. 초기화 부분 (main 함수 상단)
    let mut currents = [450, 300, 300, 300]; // 550에서 450으로 하향 조정

        // loop 외부 설정
        let mut shoulder_pos = 450u16; // 1번 (안정권 확인됨)
        let mut elbow_pos = 300u16;    // 2번 (이제 시작!)
        let mut current_pos_c3 = 300u16; // 3번 그리퍼 초기값 (중립)

    loop {

      println!("🍎 [3축 통합 테스트] 사과 포획 시퀀스 시작!");

    // 1단계: 유기적 접근 (어깨 내리고 팔꿈치 펴기)
    // C1: 450->300, C2: 300->450
    println!("  -> Step 1: 접근 중...");
    move_4axis_organic(&mut pwm, [300, 450, 300, 300], &mut currents);
    FreeRtos::delay_ms(1500);

    // 2단계: 그리퍼 작동 (사과 잡기)
    // C3: 300(중립) -> 200(닫힘/잡기)
    println!("  -> Step 2: 🦀 그리퍼 작동! (사과 고정)");
    move_arm_smooth(&mut pwm, Channel::C3, &mut currents[2], 200);
    FreeRtos::delay_ms(1000);

    // 3단계: 미세 하강 (어깨 250까지 밀착)
    println!("  -> Step 3: 표면 밀착 (C1: 250)");
    move_arm_smooth(&mut pwm, Channel::C1, &mut currents[0], 250);
    FreeRtos::delay_ms(1000);

    // 4단계: 안전 복귀 (그리퍼 열고 초기 자세로)
    println!("  -> Step 4: 그리퍼 해제 및 홈 위치 복귀");
    move_arm_smooth(&mut pwm, Channel::C3, &mut currents[2], 300); // 그리퍼 열기
    FreeRtos::delay_ms(500);
    move_4axis_organic(&mut pwm, [450, 300, 300, 300], &mut currents);
    
    println!("✅ 시퀀스 완료! 모터들이 여전히 시원한지 확인해 주세요. ㅋㅋ");
    FreeRtos::delay_ms(4000); 
        
        /*for (id, name, channel) in channels.iter() {
            println!("🔔 {}번 {} 모터 작동 테스트", id, name);

            let idx = *id as usize;

            match *id {
                /*0 => { // 베이스: 시원하게 회전
                    log::info!("🔔 0번 베이스 모터 작동");
                    move_arm_smooth(&mut pwm, *channel, &mut base_pos, 150);
                    FreeRtos::delay_ms(1000);
                    move_arm_smooth(&mut pwm, *channel, &mut base_pos, 450); 
                },
                */
                1 => { // 어깨: 너무 구부리지 않게 범위 축소 (80도 ~ 110도)
                    log::info!("🔔 1번 어깨 모터 (위쪽 Safe Zone)");
                    // 사진의 위치(400)에서 위아래로 살짝만 움직여 부하 최소화
                    move_arm_smooth(&mut pwm, *channel, &mut shoulder_pos, 400); // 위로 더 들기
                    FreeRtos::delay_ms(1200);
                    //move_arm_smooth(&mut pwm, *channel, &mut shoulder_pos, 380); // 살짝 내리기 
                },
                2 => { // 팔꿈치: 새로 추가된 관절 테스트
                    log::info!("🔔 2번 팔꿈치 모터");
                    move_arm_smooth(&mut pwm, *channel, &mut elbow_pos, 480);
                    //move_arm_smooth(&mut pwm, *channel, &mut elbow_pos, 100);
                    FreeRtos::delay_ms(1000);
                    //move_arm_smooth(&mut pwm, *channel, &mut elbow_pos, 320); 
                },
                /*3 => { // 3. 팔꿈치: 새로 추가된 관절 테스트
                   // 1. 최소 안전 각도 (예: 팔을 가볍게 굽힘)
                    log::info!("   -> 굽히기 (Safe Zone)");
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 260);
                    FreeRtos::delay_ms(1500);

                    // 2. 최대 안전 각도 (예: 팔을 가볍게 폄)
                    log::info!("   -> 펴기 (Safe Zone)");
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 340);
                    FreeRtos::delay_ms(1500);

                },*/
                _ => {}
            }
            */
            // [안정성] 테스트 후 사진 속의 그 '적당한 위치'로 매번 복귀
            /*if *id == 1 {
                move_arm_smooth(&mut pwm, *channel, &mut shoulder_pos, 550);
            } else {
                pwm.set_channel_on_off(*channel, 0, 300).unwrap();
            }*/

           

            // Motor #1번 테스트..
            /*let mut delta=240; 
            for i in 0..=8 {
                println!("🔔 1번 어깨 모터 (위쪽 Safe Zone) - {}",delta);                /// 사진의 위치(400)에서 위아래로 살짝만 움직여 부하 최소화
                move_arm_smooth(&mut pwm, Channel::C1, &mut shoulder_pos, delta); // 위로 더 들기
                delta += 20;
                FreeRtos::delay_ms(1500);
            }

            println!("✅ 최고점 도달, 잠시 휴식...");
            FreeRtos::delay_ms(2000);
            */

            // 1. 먼저 어깨(1번)를 가장 안정적인 위치인 400으로 보냅니다.
            /*move_arm_smooth(&mut pwm, Channel::C1, &mut shoulder_pos, 400);
            FreeRtos::delay_ms(1000);

            // 2. 어깨가 고정된 상태에서 팔꿈치(2번)의 가동 범위를 테스트합니다.
            let mut elbow_delta = 150; 
            for i in 0..=10 {
                println!("🔔 2번 팔꿈치 관절 테스트 중: {}", elbow_delta);
                move_arm_smooth(&mut pwm, Channel::C2, &mut elbow_pos, elbow_delta);
                elbow_delta += 20;
                FreeRtos::delay_ms(1000);
            }
            */

            // 2단계: 안전하게 초기 위치로 복귀 (400 -> 240)
            // 갑자기 툭 떨어지면 기어가 나갈 수 있으니 다시 부드럽게 내립니다.
            //println!("⬇️ 1번 어깨 안전 복귀 시작");
            //move_arm_smooth(&mut pwm, Channel::C1, &mut shoulder_pos, 240);

             // Motor #2 -- TESI code..
            /*let mut Counter = 150;
            for i in 0..=10 {
                Counter += 10;
                println!("🔔 2번 팔꿈치 모터 - {}...",Counter);
                move_arm_smooth(&mut pwm, Channel::C2, &mut elbow_pos, Counter);
                FreeRtos::delay_ms(1000);
            }
            */

        // 1번(어깨)은 상승(400), 2번(팔꿈치)은 하강(150)
        /*move_arms_simultaneous(&mut pwm, 400, 150, &mut shoulder_pos, &mut elbow_pos);
        FreeRtos::delay_ms(2000);

        // 반대로 교차: 1번 하강(240), 2번 상승(350)
        move_arms_simultaneous(&mut pwm, 240, 350, &mut shoulder_pos, &mut elbow_pos);
        FreeRtos::delay_ms(2000);    
        */

        /* 
        let mut arm: RobotArm3Axis<RefCellDevice<'_, I2cDriver<'_>>> = RobotArm3Axis { 
            current_angles: [90.0, 90.0, 90.0],_marker: PhantomData };
    
        // 사과 표면에 접근하는 3축 복합 동작 [cite: 2026-01-24, 2026-02-23]
        let apple_touch_pose = [45.0, 120.0, 30.0]; 
        // 1. 아마 위쪽 어딘가에 이렇게 선언되어 있을 겁니다. [cite: 2026-02-02]
        let delay_driver =  Delay::new(600_000_000); //Delay::new(peripherals.CPULP); // 예시
        arm.move_to_target(&mut pwm, apple_touch_pose, &delay_driver);

        println!("Done...");
        FreeRtos::delay_ms(2000);
        */

      /*  
        // 2. 다시 중립으로 (450 -> 300)
    println!("   -> 중립 복귀");
    move_arm_safe_power(&mut pwm, Channel::C3, &mut wrist_pos, 300);
    FreeRtos::delay_ms(1000);

    // 3. 반시계 방향으로 천천히 (300 -> 150)
    println!("   -> 각도 감소 (반대 방향)");
    move_arm_safe_power(&mut pwm, Channel::C3, &mut wrist_pos, 150);
    FreeRtos::delay_ms(1500);

    // 4. 최종 안전 위치로 복귀
    move_arm_safe_power(&mut pwm, Channel::C3, &mut wrist_pos, 300);
    println!("✅ 3번 관절 테스트 1주기 완료. 3초 대기...");
    FreeRtos::delay_ms(3000);
    */

    // -- 2026.03.02 TEST ---
    /* 
    println!("🔔 4번 관절(그리퍼/회전) 테스트 시작");

    // 4번 관절용 초기 위치 (중립 300 가정)
    let mut joint_4_pos = 300u16; 
    
    // 1. 부드럽게 한쪽으로 이동 (그리퍼 열기/칼날 회전)
    println!("   -> 4번 이동 (300 -> 420)");
    move_arm_safe_power(&mut pwm, Channel::C4, &mut joint_4_pos, 420);
    FreeRtos::delay_ms(1500);

    // 2. 다시 중립으로 복귀
    println!("   -> 중립 복귀 (420 -> 300)");
    move_arm_safe_power(&mut pwm, Channel::C4, &mut joint_4_pos, 300);
    FreeRtos::delay_ms(1000);

    // 3. 반대쪽으로 이동 (그리퍼 닫기/칼날 역회전)
    println!("   -> 4번 반대 이동 (300 -> 180)");
    move_arm_safe_power(&mut pwm, Channel::C4, &mut joint_4_pos, 180);
    FreeRtos::delay_ms(1500);

    // 4. 안전 위치 복귀
    move_arm_safe_power(&mut pwm, Channel::C4, &mut joint_4_pos, 300);
    println!("✅ 4번 관절 테스트 완료. 3초 후 재시작...");
    FreeRtos::delay_ms(3000); 
    */

    // 유기적으로 움직임 테스트..:
    // 준비자세..
    /*move_4axis_organic(&mut pwm, [400, 300, 300, 300], &mut currents);

    move_4axis_organic(&mut pwm, [450, 480, 250, 420], &mut currents);
    */

    // 나선향 궤적 연습..
    //run_apple_spiral_test(&mut pwm, &mut currents);

    // 사과 깎기 통합 시퀀스 (Full Sequence)
    //run_full_apple_sequence(&mut pwm, &mut currents);

       //println!("🚀 [1, 2번 집중 테스트] 1단계: 어깨(C1) 가동 범위 확인");
        // 어깨를 안전한 중립 위치(400)에서 시작
        //move_arm_smooth(&mut pwm, Channel::C1, &mut currents[0], 400);

       /*println!("🚀 [2번 모터 실전 테스트] 1번(200) 고정 + 2번 거리 조절");

    // 1단계: 1번을 200으로 먼저 숙이기 (이미 확인된 안전 지점)
    move_shoulder_safe(&mut pwm, &mut currents[0], 200);
    FreeRtos::delay_ms(1000);

    // 2단계: 2번 팔꿈치를 서서히 펴기 (300 -> 380 -> 450)
    // 380: 사과 근처 대기 위치
    println!("  -> 팔꿈치 380 이동 (접근)");
    move_arm_smooth(&mut pwm, Channel::C2, &mut currents[1], 380);
    FreeRtos::delay_ms(1500);

    // 450: 사과 표면 접촉 시도 (이때 칼날 위치를 확인하세요!)
    println!("  -> 팔꿈치 450 이동 (밀착)");
    move_arm_smooth(&mut pwm, Channel::C2, &mut currents[1], 450);
    FreeRtos::delay_ms(2000);

    // 다시 중립 위치로 복귀 (부하 감소)
    println!("🔄 안전 위치로 복귀");
    move_arm_smooth(&mut pwm, Channel::C2, &mut currents[1], 300);
    FreeRtos::delay_ms(2000); 
    */

        // 팔꿈치(2번)만 움직여서 무게 중심 변화를 관찰합니다.
    /*let elbow_targets = [250, 400];
    for target in elbow_targets.iter() {
        println!("  -> 팔꿈치 이동 목표: {}", target);
        move_arm_smooth(&mut pwm, Channel::C2, &mut currents[1], *target);
        FreeRtos::delay_ms(1000);
    }

    // 복합 동작 시에도 어깨를 420 이상 뒤로 보내지 않습니다.
    println!("🚀 [복합 테스트] 어깨(420) & 팔꿈치(300)");
    
      // head는 [index 0], tail은 [index 1, 2, 3]을 가집니다.
        let (head, tail) = currents.split_at_mut(1);

        move_arms_simultaneous(&mut pwm, 420, 300, &mut head[0], &mut tail[1]);
        */

        // 사과 접근 시에도 너무 뒤에서 시작하지 않도록 조정
    //move_shoulder_safe(&mut pwm, &mut currents[0], 320);

        // 2번 팔꿈치(C2)를 고정된 어깨 상태에서 천천히 움직여 봅니다.
        // RDS3225의 반응성을 보기 위해 스텝을 잘게 쪼갭니다.
        

        /* 
        println!("🚀 [1, 2번 집중 테스트] 2단계: 복합 유기적 이동");
        // 어깨와 팔꿈치가 동시에 움직일 때 전원(7.4V) 안정성 체크 [cite: 2026-02-23]
        // 1번 상승(500) & 2번 하강(200)

        // head는 [index 0], tail은 [index 1, 2, 3]을 가집니다.
        let (head, tail) = currents.split_at_mut(1);

        move_arms_simultaneous(&mut pwm, 500, 200, &mut head[0], &mut tail[1]);

        FreeRtos::delay_ms(2000); 

        // 1번 하강(300) & 2번 상승(400)
        move_arms_simultaneous(&mut pwm, 300, 400, &mut head[0], &mut tail[1]);
        FreeRtos::delay_ms(2000);

        println!("✅ 한 주기가 완료되었습니다. 모터 발열 상태를 확인하세요.");
        */
        FreeRtos::delay_ms(3000);


    } 
}

// 부드러운 이동을 위한 헬퍼 함수
fn move_to_target(driver: &mut LedcDriver, current: &mut u32, target: u32) -> anyhow::Result<()> {
    while *current != target {
        if *current < target {
            *current += 2; // 매우 부드러운 가속
        } else {
            *current -= 2;
        }
        driver.set_duty(*current)?;
        FreeRtos::delay_ms(15); // 안정적인 스텝 간격
    }
    Ok(())
}

// PCA9685 전용 부드러운 이동 로직 (예시)
fn move_pca_smooth(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>, 
    channel: Channel, 
    from: u16, 
    to: u16
) {
    let step = if from < to { 1 } else { -1 };
    let mut current = from;
    
    while current != to {
        current = (current as i16 + step) as u16;
        pwm.set_channel_on_off(channel, 0, current).unwrap();
        FreeRtos::delay_ms(10); // 안정성을 위해 조금씩 이동 [cite: 2026-02-13]
    }
}

// [안정성 강화] 목표 각도까지 가속/감속하며 이동하는 함수 [cite: 2026-02-13]
fn move_arm_smooth(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    channel: Channel,
    current_pos: &mut u16,
    target_pos: u16,
) {
    let diff = (target_pos as i32 - *current_pos as i32).abs();
    if diff == 0 { return; }

    let steps = 40; // 분할 단계 (클수록 더 부드러움)
    
    for i in 1..=steps {
        // Sine 함수를 이용한 부드러운 보간 (0.0 ~ 1.0)
        let t = i as f32 / steps as f32;
        let ease_step = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;
        
        let next_pos = (*current_pos as f32 + (target_pos as i32 - *current_pos as i32) as f32 * ease_step) as u16;
        
        pwm.set_channel_on_off(channel, 0, next_pos).unwrap();
        
        // 이동 중 물리적 부하 분산을 위한 짧은 대기 [cite: 2026-02-13]
        FreeRtos::delay_ms(15); 
    }
    
    *current_pos = target_pos;
}

// [안정성 강화] 1번(상승) & 2번(하강) 교차 동시 제어 로직
fn move_arms_simultaneous(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    target_1: u16,
    target_2: u16,
    current_1: &mut u16,
    current_2: &mut u16,
) {
    let steps = 50; // 동시 동작이므로 단계를 더 세분화해서 부하 분산
    
    let start_1 = *current_1 as f32;
    let start_2 = *current_2 as f32;
    let diff_1 = target_1 as f32 - start_1;
    let diff_2 = target_2 as f32 - start_2;

    println!("🚀 동시 동작 시작: [1번] {}->{}, [2번] {}->{}", *current_1, target_1, *current_2, target_2);

    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        // Sine Ease-in-out 보간 (부드러운 가속/감속) [cite: 2026-02-13]
        let ease = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;

        let next_1 = (start_1 + diff_1 * ease) as u16;
        let next_2 = (start_2 + diff_2 * ease) as u16;

        // 두 모터 신호를 거의 동시에 쏴줍니다.
        pwm.set_channel_on_off(Channel::C1, 0, next_1).unwrap();
        pwm.set_channel_on_off(Channel::C2, 0, next_2).unwrap();

        // 7.4V 전력이 안정적이더라도, 두 모터가 동시에 움직일 때의 
        // 전압 강하를 방지하기 위해 미세한 딜레이를 줍니다.
        FreeRtos::delay_ms(20); 
    }

    *current_1 = target_1;
    *current_2 = target_2;
    println!("✅ 동시 동작 완료!");
}

// 기존 move_arm_smooth를 조금 더 '신중하게' 개선
fn move_arm_safe_power(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    channel: Channel,
    current_pos: &mut u16,
    target_pos: u16,
) {
    let steps = 100; // 단계를 더 세분화하여 7.4V의 반동을 억제
    let start = *current_pos as f32;
    let diff = target_pos as f32 - start;

    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        // 더 깊은 Sine 곡선으로 시작과 끝을 아주 부드럽게 처리
        let ease = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;
        let next_pos = (start + diff * ease) as u16;

        pwm.set_channel_on_off(channel, 0, next_pos).unwrap();
        
        // 고전압 모터의 빠른 반응성에 맞춰 딜레이를 최적화 (10~15ms)
        FreeRtos::delay_ms(12); 
    }
    *current_pos = target_pos;
}

// [안정성 & 유기적 제어] 1, 2, 3, 4번 관절 통합 동시 제어
fn move_4axis_organic(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    targets: [u16; 4],      // [target_1, target_2, target_3, target_4]
    currents: &mut [u16; 4], // [curr_1, curr_2, curr_3, curr_4]
) {
    let steps = 120; // 유기적인 움직임을 위해 단계를 더 세분화합니다. [cite: 2026-02-13]
    
    let starts = [currents[0] as f32, currents[1] as f32, currents[2] as f32, currents[3] as f32];
    let diffs = [
        targets[0] as f32 - starts[0],
        targets[1] as f32 - starts[1],
        targets[2] as f32 - starts[2],
        targets[3] as f32 - starts[3],
    ];

    println!("🚀 4축 유기적 협업 동작 시작...");

    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        // Sine Ease-in-out으로 모든 관절의 가속/감속을 통일
        let ease = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;

        let n1 = (starts[0] + diffs[0] * ease) as u16;
        let n2 = (starts[1] + diffs[1] * ease) as u16;
        let n3 = (starts[2] + diffs[2] * ease) as u16;
        let n4 = (starts[3] + diffs[3] * ease) as u16;

        // PCA9685를 통해 거의 동시에 명령 하달
        pwm.set_channel_on_off(Channel::C1, 0, n1).unwrap();
        pwm.set_channel_on_off(Channel::C2, 0, n2).unwrap();
        pwm.set_channel_on_off(Channel::C3, 0, n3).unwrap();
        pwm.set_channel_on_off(Channel::C4, 0, n4).unwrap();

        // 7.4V 전원의 안정성을 위해 미세 딜레이 조정 [cite: 2026-02-13, 2026-02-23]
        FreeRtos::delay_ms(15); 
    }

    // 현재 위치 업데이트
    for i in 0..4 { currents[i] = targets[i]; }
    println!("✅ 유기적 이동 완료!");
}

fn run_apple_spiral_test(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 4]
) {
    println!("🍎 사과 깎기 나선형 궤적 연습 시작!");

    // 나선형 궤적 좌표 설정 [1번 어깨, 2번 팔꿈치, 3번 손목, 4번 회전/그리퍼]
    // 주의: 실제 기구학적 구조에 따라 각도(Pulse) 값은 조정이 필요합니다. [cite: 2026-02-21]
    let spiral_path = [
        [450, 400, 300, 300], // 1단계: 사과 상단 접근
        [460, 420, 310, 350], // 2단계: 약간 회전하며 하강 시작
        [470, 440, 320, 400], // 3단계: 중간 지점
        [480, 460, 330, 450], // 4단계: 하단부 도달
        [450, 300, 300, 300], // 5단계: 안전하게 후퇴
    ];

    for (i, target) in spiral_path.iter().enumerate() {
        println!("📍 궤적 단계 {}: {:?}", i + 1, target);
        
        // 단계별 이동: 사과를 깎을 때는 더 천천히 움직이도록 설정 가능 [cite: 2026-02-13]
        move_4axis_organic(pwm, *target, currents);
        
        // 각 단계 사이의 아주 짧은 대기 (연속성을 위해 짧게 설정)
        FreeRtos::delay_ms(200);
    }

    println!("✅ 나선형 궤적 연습 완료!");
}

fn run_full_apple_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 4]
) {
    println!("🚀 [시퀀스 시작] 사과 깎기 마술을 시작합니다!");

    // 1. 위쪽 Safe Zone으로 이동하여 공간 확보
    println!("Step 1: 공간 확보 중...");
    move_4axis_organic(pwm, [420, 300, 300, 300], currents);
    FreeRtos::delay_ms(1000);

    // 2. 사과 파지 위치로 접근 (그리퍼 열기)
    println!("Step 2: 사과 접근 및 그리퍼 개방");
    move_4axis_organic(pwm, [450, 400, 300, 420], currents);
    FreeRtos::delay_ms(1500);

    // 3. 사과 잡기 (그리퍼 닫기)
    println!("Step 3: 사과 고정!");
    move_4axis_organic(pwm, [450, 400, 300, 250], currents); 
    FreeRtos::delay_ms(2000); // 고정 확인을 위한 충분한 시간

    // 4. 나선형 궤적 (깎기 동작)
    println!("Step 4: 나선형 깎기 궤적 시작...");
    let spiral_steps = [
        [460, 420, 310, 250],
        [470, 440, 320, 250],
        [480, 460, 330, 250],
    ];
    for target in spiral_steps.iter() {
        move_4axis_organic(pwm, *target, currents);
        FreeRtos::delay_ms(300);
    }

    // 5. 완료 후 안전하게 복귀
    println!("Step 5: 작업 완료, 원위치 복귀");
    move_4axis_organic(pwm, [420, 300, 300, 300], currents);
    
    println!("✅ [시퀀스 종료] 오늘 테스트 성공적!");
}

// 1번 모터 전용: 각도 제한 및 더 부드러운 이동
fn move_shoulder_safe(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    current_pos: &mut u16,
    target_pos: u16,
) {
    // [안전 장치] 300~500 사이로 강제 제한 (기구 파손 방지)
    let safe_target = target_pos.clamp(200, 420); 
    
    // 관성을 줄이기 위해 단계를 100으로 대폭 늘림 [cite: 2026-02-13]
    let steps = 120; 
    let start = *current_pos as f32;
    let diff = safe_target as f32 - start;

    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let ease = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;
        let next_pos = (start + diff * ease) as u16;

        pwm.set_channel_on_off(Channel::C1, 0, next_pos).unwrap();
        
        // RDS3225의 고토크 반동을 억제하기 위한 딜레이 [cite: 2026-02-13, 2026-02-23]
        FreeRtos::delay_ms(15); 
    }
    *current_pos = safe_target;
}