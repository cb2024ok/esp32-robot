use std::cell::RefCell;
use std::f32::consts::PI;
use std::marker::PhantomData;
use std::rc::Rc;
use std::sync::Arc;
use std::time::Duration;

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
use esp32_nimble::utilities::mutex::Mutex;
//use pwm_pca9685::{Address, Pca9685,Channel};
use pwm_pca9685::*;
use esp_idf_hal::ledc::*;
// 자유의 날개 프로젝트 실시
use esp32_nimble::{uuid128, BLEAdvertisementData, BLEDevice, NimbleProperties};




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


const GRIPPER_OPEN: u32 = 480;  // 시원하게 열기
const GRIPPER_CLOSE: u32 = 185; // 안정적으로 닫기
const GRIPPER_IDLE: u32 = 300;  // 기본 자세

// 1. 설계 상수 정의 (마술사님의 관찰 데이터 기반)
const PHYSICAL_MIN: f32 = 175.0; // 물리적 절대 하한선
const PHYSICAL_MAX: f32 = 597.0; // 물리적 절대 상한선
const SAFE_WORK_MIN: f32 = 180.0; // 우리가 정한 작업 하한선 (Y=57 기준)

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

fn angle_to_pulse(angle: f32) -> u16 {
        (SERVO_MIN as f32 + (angle / 180.0) * (SERVO_MAX - SERVO_MIN) as f32) as u16
    }

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    let peripherals = Peripherals::take()?;

   println!("🚀 [테스트] I2C를 제외하고 PS2 컨트롤러만 시작합니다...");

   // LEDC 설정 (50Hz)
    /*
    let timer_config = config::TimerConfig::new()
        .frequency(Hertz(50).into())
        .resolution(Resolution::Bits14);
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config)?;

    let mut motor_0 = LedcDriver::new(
        peripherals.ledc.channel0,
        &timer,
        peripherals.pins.gpio15, // ESP32-C6 핀맵 확인 후 수정
    )?;
    */

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
    //let i2c_ref_cell =  RefCell::new(i2c);
    //let i2c_device = RefCellDevice::new(&i2c_ref_cell);
    //let i2c_clone = Arc::clone(&i2c_ref_cell); //i2c_ref_cell.clone();

    //let mut i2c_driver = i2c_clone.lock();
    //let mut i2c_driver = i2c_clone.lock();

    //let i2c_bus = shared_bus::BusManagerSimple::new(i2c);
    //let i2c_device = i2c_bus.acquire_i2c(); 

    //let i2c = Arc::new(Mutex::new(i2c));
    //let i2c_clone = i2c.clone();

    //let mut i2c_guard = i2c_clone.lock();
    //let pwm = Pca9685::new(i2c, SlaveAddr::default());
    //let mut pwm = pwm;
    //pwm.set_prescale(100).unwrap(); // ~60Hz for servos
    //pwm.enable().unwrap();
    

    // 2. PCA9685용 가상 I2C 핸들을 만듭니다. (소유권 문제 해결)
    //let pwm_i2c =  RefCellDevice::new(&i2c_ref_cell);// i2c_bus.acquire_i2c();

    let mut pwm = Pca9685::new(i2c, Address::from(0x60))
    //let mut pwm = Pca9685::new(pwm_i2c, Address::from(0x60))
     .map_err(|_| anyhow::anyhow!("PCA9685 초기화 실패"))?;
    
    pwm.set_prescale(121).unwrap();
    pwm.enable().unwrap();

    let shared_pwm = Arc::new(Mutex::new(pwm));
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

    //println!("=== 관절로봇 테스트 시작 ===");

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
    let mut currents = [450, 300, 300, 300, 300]; // 550에서 450으로 하향 조정

        // loop 외부 설정
        let mut shoulder_pos = 450u16; // 1번 (안정권 확인됨)
        let mut elbow_pos = 300u16;    // 2번 (이제 시작!)
        let mut current_pos_c3 = 300u16; // 3번 그리퍼 초기값 (중립)
        let mut input_target = 0u32 ;
        let mut current_angle = 300u32;

          // BLE Device init
    let ble_device = BLEDevice::take();
    let server = ble_device.get_server();

    // for connection..
    server.on_connect(|_server, desc| {
        println!("BLE -- iPhone F-22 연결됨!: {:?}", desc);
    });

    // 서비스 생성
    let service = server.create_service(uuid128!("fafafafa-abcd-4321-abcd-fafafafafafa"));

    // 3. 특성
    let control_characteristic = service.lock().create_characteristic(
        uuid128!("12345678-1234-5678-1234-567812345678"),
        NimbleProperties::WRITE | NimbleProperties::READ,
    );

    let pwm_clone = Arc::clone(&shared_pwm);

    // 데이터 수신시 콜백
    control_characteristic.lock().on_write(move |args| {
        let packet = args.recv_data();
        //println!("BLE DATA-> {:?}",data);
        if packet.len() >= 5 && packet[0] == 0xAA {
            let motor_id = packet[1];
            let x_angle = packet[2];
            let y_angle = packet[3];
            let checksum = packet[4];

            
             println!("🚀 [TA System] Received: ID={}, X={}, Y={}, CS={}", 
                    motor_id, x_angle, y_angle, checksum);
            
            // 여기에 서보 PWM 제어 로직 연결
                
                // PCA9685 채널 0 모터 제어로직 연결
                // 1번 모터(Y축)에 대한 정밀 보정
                let pulse: u16 = if motor_id == 0 || motor_id >= 4 {
                    angle_to_pulse(x_angle as f32)
                } else {
                    // 255를 초과하는 400도 제어를 위해 스케일링 적용
                    //let y_scaled = (y_angle as f32 / 255.0) * 400.0;
                    //let y_scaled = y_angle as f32 + 10.0f32;
                    calculate_pulse(y_angle.into()) as u16 //y_angle as f32 + 10.0f32;
                    //angle_to_pulse(y_scaled)
                };

                // HOME 명령 수행
                if motor_id == 0xFF {
                    let home_angles: [f32; 3] = [90.0, 133.0, 152.0];
                    let channels = [Channel::C1, Channel::C2, Channel::C0];

                       let mut pwm_inner = pwm_clone.lock() ;
                       for (i,&ch) in channels.iter().enumerate() {
                        // 2. 각도를 펄스로 변환 (핵심!)
                            let target_pulse = angle_to_pulse(home_angles[i]);
                            move_smoothly(&mut *pwm_inner, channels[i], &mut current_positions[i as usize], target_pulse as u16);
                            current_positions[i] = target_pulse as u16;
                            FreeRtos::delay_ms(50);
                       } 
                } else {

                println!("calc pulse value: {}",pulse); 
                //let mut pwm = pwm_clone.lock();
                // 2. ID에 따른 PCA9685 채널 결정
                let target_channel = match motor_id {
                    0 => Some(Channel::C0),
                    1 => Some(Channel::C1),
                    2 => Some(Channel::C2),
                    3 => Some(Channel::C3),
                    4 => Some(Channel::C4),
                    5 => Some(Channel::C5),
                    _ => {
                        println!("⚠️ 경고: 정의되지 않은 모터 ID: {}", motor_id);
                        None
                    }
                };

                // 3. 해당 채널이 있을 때만 구동
                if let Some(channel) = target_channel {
                    let mut pwm = pwm_clone.lock();
                    //pwm.set_channel_on_off(channel, 0, pulse).unwrap();

                    if let Err(e) = pwm.set_channel_on_off(channel, 0, pulse) {
                           println!("🚨 I2C Write Failed: {:?}", e); // 여기서 에러가 찍히면 전원/연결 문제!
                    }
                    
                    // 안정성 확보를 위한 지연 (기존 철학 유지)
                    FreeRtos::delay_ms(10); 
                }
              }
                //pwm.set_channel_on_off(Channel::C0, 0, pulse).unwrap();
                //FreeRtos::delay_ms(10); // 안정성을 위해 조금씩 이동 [cite: 2026-02-13]
            }


            
    });

    // 광고시작
    let advertising = ble_device.get_advertising();

    let mut ad_data: BLEAdvertisementData = BLEAdvertisementData::new();
    ad_data.name("Magician_Su57_P4");
    advertising.lock().set_data(&mut ad_data).unwrap();

    advertising.lock().start().unwrap();
    log::info!("BLE 광고중... ipHone에서 연결을 기다림..!");

    loop {

        //run_spiral_peeling_sequence(&mut pwm, &mut currents);
        //run_cool_spiral_sequence(&mut pwm, &mut currents);
        //run_c5_solo_performance(&mut pwm, &mut currents);
        // RDS3225의 안전 범위를 170~550으로 제한하는 예시
        // 1. 처음 시작할 때
        /* 
        let mut current_angle: u32 = 300; // 초기 구동 위치 (중립)

        // 2. 대기 모드로 이동시키고 싶을 때
        let input_target = GRIPPER_IDLE; // GRIPPER_IDLE을 300~350 정도로 설정

        // 3. 안전 범위 체크 후 이동
        let safe_target = input_target.clamp(GRIPPER_CLOSE, GRIPPER_OPEN);
            run_c5_power_test(&mut pwm,&mut current_angle, safe_target);

        // 테스트용: 180과 480을 왔다갔다 하게 강제 할당
        let test_target = if current_angle < 300 { 480 } else { 180 };
    
        let safe_target = test_target.clamp(GRIPPER_CLOSE, GRIPPER_OPEN);
        run_c5_power_test(&mut pwm, &mut current_angle, safe_target);
       */
    // 한 번 동작 후 잠시 대기
    FreeRtos::delay_ms(1000);
        
        /* 
        // [마술사 전용] 1번(250) + 2번 고정 시연 시퀀스
println!("✨ [시연 시작] 1번(250) 황금 지점 고정 모드!");

// 1. 초기 자세: 1번을 250으로, 2번을 300으로 꽉 잡아 고정
// [C1, C2, C3, C4, C5] 순서 (C3은 사과를 잡은 상태로 가정)
let mut target_pose = [250, 300, 200, 300, 300]; 
move_5axis_organic(&mut pwm, target_pose, &mut currents);
FreeRtos::delay_ms(1500);

// 2. 5번 모터(C5) '진입각' 테스트 (300 -> 500)
// 1, 2번이 고정되어 있어 5번의 움직임이 아주 잘 보일 겁니다.
println!("  -> 5번 모터(C5) 각도 크게 변화 중... (300 to 500)");
target_pose[4] = 500;
move_5axis_organic(&mut pwm, target_pose, &mut currents);
FreeRtos::delay_ms(1000);

// 3. 4번(회전)하며 5번(보정)하는 '실전 깎기' 시뮬레이션
for i in 0..5 {
    let next_c4 = 250 + (i * 50); // 250, 300, 350, 400, 450
    let next_c5 = 500 - (i * 40); // 500, 460, 420, 380, 340 (변화폭 확대)
    
    println!("  -> [동작] C4(회전): {}, C5(각도보정): {}", next_c4, next_c5);
    move_5axis_organic(&mut pwm, [250, 300, 200, next_c4, next_c5], &mut currents);
    FreeRtos::delay_ms(300);
}

// 4. 안전하게 복귀
println!("✅ 시연 완료! 기둥(1,2번)의 안정성을 확인하세요.");
move_5axis_organic(&mut pwm, [250, 300, 300, 300, 300], &mut currents);
*/

        //run_vertical_magic_sequence(&mut pwm, &mut currents);
        //run_fixed_pillar_sequence(&mut pwm, &mut currents);

        /* 
        // [긴급 진단] 1번 모터(C1)가 신호를 받는지 확인하는 코드
        println!("📢 1번 모터 강제 기상 테스트 시작!");

        // 1. 1번 모터만 '눕히기' (값: 250)
        println!("  -> 1번: 250으로 이동 (눕히기)");
        move_arm_smooth(&mut pwm, Channel::C1, &mut currents[0], 250);
        FreeRtos::delay_ms(2000); // 충분히 관찰할 시간

        // 2. 1번 모터만 '세우기' (값: 500)
        println!("  -> 1번: 500으로 이동 (세우기)");
        move_arm_smooth(&mut pwm, Channel::C1, &mut currents[0], 500);
        FreeRtos::delay_ms(2000);

        // 3. 1번 모터 '중립' (값: 375)
        println!("  -> 1번: 375로 복귀 (중립)");
        move_arm_smooth(&mut pwm, Channel::C1, &mut currents[0], 375);
        FreeRtos::delay_ms(1000);

        println!("✅ 1번이 움직였나요? 안 움직였다면 선을 바꿔 꽂아야 합니다!");
        */

        /* 
println!("✨ [마술 시연] 사과 깎기 5축 통합 모드 시작!");

// 초기 위치 (C1~C5)
let mut currents = [450, 300, 300, 300, 300]; 

// 1. 접근: 어깨 내리고 손목 세우기
move_5axis_organic(&mut pwm, [300, 450, 200, 300, 400], &mut currents);
FreeRtos::delay_ms(1000);

// 2. 절삭 각도 조절: 5번 모터(C5)를 400 -> 350으로 조절하여 칼날 진입
println!("  -> 칼날 각도 조정 중...");
move_arm_smooth(&mut pwm, Channel::C5, &mut currents[4], 350); 
FreeRtos::delay_ms(500);

// 3. 회전하며 하강 (유기적 동작의 핵심)
move_5axis_organic(&mut pwm, [350, 480, 200, 450, 320], &mut currents);
println!("✅ 시연 완료! 장치가 뜨거워지지 않았는지 확인해 보세요.");
*/

      /*println!("🍎 [3축 통합 테스트] 사과 포획 시퀀스 시작!");

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
    */
        
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

/// 마술사님의 보정 로직이 포함된 펄스 계산기
/// calculate_pulse (2026.03.11) 적용
/// 
fn calculate_pulse(y_angle: u32) -> u32 {
    // 1단계: 오프셋 보정 (+10.0)
    //let y_scaled = y_angle as f32 + 10.0;

    // 2단계: 펄스 변환 (기존 계산식 유지)
    // 펄스 범위 252 ~ 597 사이의 매핑
    //let pulse = 252.0 + (y_scaled - 31.0) * 2.5;

    let y_scaled = (y_angle as f32).clamp(0.0, 89.0);
    println!("1 => y_clamped: {}", y_scaled);

    // 3단계: 안전 보호 로직 적용 (Clamp & Limit)
    // 물리적 한계를 벗어나지 않도록 방어하고, 작업 최소치를 넘는지 체크
    //let final_pulse = pulse.clamp(PHYSICAL_MIN, PHYSICAL_MAX);
    //let y_scaled = (y_angle as f32 + 10.0).clamp(0.0, 180.0);

    //println!("1 => y_scaled: {}",y_scaled);

    // 1455가 나오던 식을 다시 500~600대 안전 구역으로 매핑
    // 예: Y=129일 때 펄스가 너무 높다면 나눗셈으로 범위를 줄입니다.
    //let pulse = 120.0 + (y_scaled * 1.5);
    //let pulse: f32 = 120.0 + (y_scaled * 3.0);
    //let pulse: f32 = 100.0 + (y_scaled * 4.5);
    let pulse = 150.0 + (y_scaled * 5.0);   // ← 이 계수(5.0)를 테스트하면서 조정
    //let pulse = 200.0 + (y_scaled * 3.0);
    println!("2 => pulse: {}",pulse);

    // 최종 보호막: 600을 절대 넘기지 않게 합니다.
    //let final_pulse = pulse.clamp(250.0, 600.0) as f32;
    // 이렇게 하면 모터가 반응 없는 허공에 총을 쏠 일이 없습니다!
    //let final_pulse = pulse.clamp(180.0, 350.0) as f32;
    //let final_pulse = pulse.clamp(200.0, 650.0) as f32;
    let final_pulse = pulse.clamp(150.0, 650.0) as f32;
    println!("3. => final_pulse: {}",final_pulse);

    /* 비교 로직 개선 */
    if final_pulse < 180.0 {
        println!("⚠️ [Low Warning] 가동 범위 미달: {}", final_pulse);
    //} else if final_pulse > 400.0 {
    } else if final_pulse > 620.0 {
        println!(
            "⚠️ [High Warning] 작업 영역 초과(500 이상): {}",
            final_pulse
        );
    }

    final_pulse as u32
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

// 5축 유기적 제어 함수로 업그레이드
// [안정성 & 정밀도] 1~5번 관절 통합 시연 제어
fn move_5axis_organic(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    targets: [u16; 5],      
    currents: &mut [u16; 5], 
) {
    let steps = 150; // 5축이 동시에 움직이므로 단계를 더 세분화 (부하 분산) [cite: 2026-02-13]
    
    let starts = [
        currents[0] as f32, currents[1] as f32, 
        currents[2] as f32, currents[3] as f32,
        currents[4] as f32
    ];
    let diffs = [
        targets[0] as f32 - starts[0],
        targets[1] as f32 - starts[1],
        targets[2] as f32 - starts[2],
        targets[3] as f32 - starts[3],
        targets[4] as f32 - starts[4],
    ];

    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let ease = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;

        // PCA9685 각 채널에 부드러운 펄스 전송
        pwm.set_channel_on_off(Channel::C1, 0, (starts[0] + diffs[0] * ease) as u16).unwrap();
        pwm.set_channel_on_off(Channel::C2, 0, (starts[1] + diffs[1] * ease) as u16).unwrap();
        pwm.set_channel_on_off(Channel::C3, 0, (starts[2] + diffs[2] * ease) as u16).unwrap();
        pwm.set_channel_on_off(Channel::C4, 0, (starts[3] + diffs[3] * ease) as u16).unwrap();
        pwm.set_channel_on_off(Channel::C5, 0, (starts[4] + diffs[4] * ease) as u16).unwrap();

        FreeRtos::delay_ms(15); // RDS3225의 안정적인 반응을 유도 [cite: 2026-02-23]
    }

    for i in 0..5 { currents[i] = targets[i]; }
}

// 1번 모터를 수직으로 고정하고 2~5번을 유기적으로 움직이는 마술 시퀀스
fn run_vertical_magic_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("✨ [직립 시연] 1번 모터 Vertical 고정 및 5축 협업 시작!");

    // 1. 준비 자세: 1번을 수직(480)으로 세우고 모든 관절 정렬
    // [C1, C2, C3, C4, C5] -> [직립, 접기, 중립, 정면, 수평]
    move_5axis_organic(pwm, [480, 250, 300, 300, 300], currents);
    FreeRtos::delay_ms(2000);

    // 2. 접근 및 5번 모터 강조: 5번(칼날 각도)을 크게 움직여 확인
    println!("  -> 5번 모터(C5) 각도 가시성 테스트 (300 -> 450)");
    move_5axis_organic(pwm, [480, 350, 200, 300, 450], currents);
    FreeRtos::delay_ms(1000);

    // 3. 나선형 회전 시뮬레이션 (4번 회전 + 5번 보정)
    for pos in (300..=450).step_by(50) {
        println!("  -> 회전(C4): {}, 각도보정(C5): {}", pos, 450 - (pos/10));
        move_5axis_organic(pwm, [480, 380, 200, pos, 450 - (pos/10)], currents);
        FreeRtos::delay_ms(500);
    }

    // 4. 안전 복귀
    println!("  -> 시연 종료 및 홈 위치 복귀");
    move_5axis_organic(pwm, [480, 250, 300, 300, 300], currents);
}

fn run_fixed_pillar_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("🚀 [고정 시연] 1, 2번 기둥 모드! (안정성 극대화)");

    // 1. 기둥 세우기 (1번: 480, 2번: 200 정도로 바짝 세움)
    // C1: 480(어깨), C2: 200(팔꿈치), C3: 300, C4: 300, C5: 300
    move_5axis_organic(pwm, [480, 200, 300, 300, 300], currents);
    FreeRtos::delay_ms(1500);

    // 2. 5번 모터 "생존 확인" 및 진입각 조절 (300 -> 450)
    // 1, 2번이 고정되어 있으므로 5번의 움직임이 훨씬 잘 보일 겁니다.
    println!("  -> 5번 모터(C5) 진입각 크게 조정 중...");
    move_5axis_organic(pwm, [480, 200, 300, 300, 450], currents);
    FreeRtos::delay_ms(1000);

    // 3. 4번(회전) 시연: 기둥은 고정된 채 손목만 뱅글뱅글
    for pos in (250..=450).step_by(40) {
        // 5번(C5)도 4번의 위치에 따라 연동하여 크게 움직이도록 설정
        let c5_target = 450 - (pos / 4); // 변화폭을 더 키웠습니다!
        println!("  -> [고정상태] 4번 회전: {}, 5번 틸트: {}", pos, c5_target);
        
        move_5axis_organic(pwm, [480, 200, 300, pos, c5_target as u16], currents);
        FreeRtos::delay_ms(400);
    }

    println!("✅ 고정 시연 완료! 1, 2번이 버텨주니 훨씬 안정적이네요. ㅋㅋ");
}

fn run_perfect_stable_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("✨ [마술사 전용 모드] 1번(250) 고정! 안정성 극대화 시연 시작");

    // 1. 황금 밸런스 자세 잡기
    // C1: 250(마술사님의 Pick!), C2: 300(안정적인 수직), C3: 300(그리퍼 중립)
    move_5axis_organic(pwm, [250, 300, 300, 300, 300], currents);
    FreeRtos::delay_ms(1500);

    // 2. 5번 모터(C5)의 화려한 등장 (변화폭 확대)
    // 1번이 250으로 단단히 버텨주니 5번을 더 역동적으로 움직여도 됩니다.
    println!("  -> 5번 모터 정밀 각도 가동 (300 -> 500)");
    move_5axis_organic(pwm, [250, 300, 300, 300, 500], currents);
    FreeRtos::delay_ms(1000);

    // 3. 4번(회전)과 5번(보정)의 콤비네이션 깎기 시연
    for pos in (200..=500).step_by(60) {
        // 4번 회전에 맞춰 5번도 춤추듯 움직이게 설정
        let c5_dynamic = 500 - (pos / 2); 
        println!("  -> [안정모드] 4번 회전: {}, 5번 보정: {}", pos, c5_dynamic);
        
        move_5axis_organic(pwm, [250, 300, 300, pos, c5_dynamic as u16], currents);
        FreeRtos::delay_ms(300); // 1번이 안정적이니 딜레이를 살짝 줄여 리드미컬하게!
    }

    // 4. 안전한 마무리
    move_5axis_organic(pwm, [250, 300, 300, 300, 300], currents);
    println!("✅ 250 지점에서 시연 완료! 확실히 소음과 진동이 줄어들었죠? ㅋㅋ");
}

fn run_spiral_peeling_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("🌀 [실전 마법] 나선형 하강 깎기 시작!");

    // 시작 위치: 1번(250), 2번(250) - 사과 꼭대기
    move_5axis_organic(pwm, [250, 250, 200, 250, 480], currents);
    FreeRtos::delay_ms(1000);

    // 나선형 궤적 (5단계로 나누어 하강)
    for i in 0..6 {
        let down_step = 250 + (i * 20);   // 2번(팔꿈치)을 조금씩 내려 하강
        let rotate_step = 250 + (i * 40); // 4번(손목) 회전
        let angle_fix = 480 - (i * 25);   // 5번(칼날) 곡률에 맞춰 각도 보정

        println!("  -> 하강: {}, 회전: {}, 각도: {}", down_step, rotate_step, angle_fix);
        
        // 1번(250)은 절대 고정! (우리의 약속)
        move_5axis_organic(pwm, [250, down_step, 200, rotate_step, angle_fix], currents);
        FreeRtos::delay_ms(400);
    }

    println!("✅ 나선형 시연 종료! 모양이 좀 나오나요? ㅋㅋ");
}

fn run_cool_spiral_sequence(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("❄️ [쿨링 모드] 4번 모터 부하를 줄이며 나선형 깎기 시작!");

    let mut current_pose = [250, 250, 200, 250, 480];
    move_5axis_organic(pwm, current_pose, currents);
    FreeRtos::delay_ms(1000);

    // 단계를 8단계로 더 쪼개서 각 단계의 회전폭을 줄입니다.
    for step in 1..=8 {
        let next_c2 = 250 + (step * 15);   // 하강폭 조절
        let next_c4 = 250 + (step * 30);   // 회전폭 축소 (열 발생 억제)
        let next_c5 = 480 - (step * 20);   // 각도보정

        move_5axis_organic(pwm, [250, next_c2, 200, next_c4, next_c5], currents);
        
        // 4번 모터가 위치를 잡고 잠시 숨을 고를 시간을 줍니다.
        FreeRtos::delay_ms(550); // 400ms -> 550ms로 소폭 증가
    }

    println!("✅ 시연 완료! 4번 모터를 만져보세요. 조금 더 시원해졌나요? ㅋㅋ");
    move_5axis_organic(pwm, [250, 250, 300, 300, 300], currents);
}

fn run_c5_solo_performance(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("✨ [마지막 테스트] 5번 모터(C5) 단독 틸트 시연!");

    // 1. 모든 축 고정 (1번은 당연히 250!)
    // [C1:250, C2:300, C3:300, C4:350, C5:300]
    let base_pose = [250, 300, 300, 350, 300];
    move_5axis_organic(pwm, base_pose, currents);
    FreeRtos::delay_ms(1000);

    // 2. 5번 모터만 왕복 (300 -> 550 -> 300)
    for target_c5 in (300..=550).step_by(25) {
        println!("  -> 5번 각도 조절 중: {}", target_c5);
        move_5axis_organic(pwm, [250, 300, 300, 350, target_c5], currents);
        FreeRtos::delay_ms(200);
    }

    for target_c5 in (300..=550).rev().step_by(25) {
        println!("  -> 5번 원위치 복귀 중: {}", target_c5);
        move_5axis_organic(pwm, [250, 300, 300, 350, target_c5], currents);
        FreeRtos::delay_ms(200);
    }

    println!("✅ 5번 모터 단독 시연 완료! 이제 모든 마법 준비가 끝났습니다. ㅋㅋ");
}

/*fn run_c5_power_test(
    pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>,
    currents: &mut [u16; 5]
) {
    println!("⚡ [파워 테스트] 5번 모터(C5) 출력 강화 시연!");

    // 1번(250)과 다른 축들은 '최소 유지' 전력으로 고정
    let standby_pose = [250, 300, 300, 350, 300];
    move_5axis_organic(pwm, standby_pose, currents);
    //move_smoothly(standby_pose, target_angle);
    FreeRtos::delay_ms(1000);

    // 5번 모터의 가동 범위를 '공격적'으로 확장 (200 -> 550)
    // 단계별 이동 폭을 키워(50단위) 더 역동적으로 움직이게 합니다.
    for target_c5 in (200..=550).step_by(50) {
        println!("  -> 5번 파워 가동: {}", target_c5);
        move_5axis_organic(pwm, [250, 300, 300, 350, target_c5], currents);
        
        // 너무 느리면 힘이 없어 보일 수 있으니 딜레이를 살짝 줄입니다.
        FreeRtos::delay_ms(250); 
    }

    println!("✅ 5번 파워 테스트 종료! 이제 좀 힘차게 움직이나요? ㅋㅋ");
}
*/

// 간단한 서보 이동 부드럽게 만들기 예시 (의사 코드)
fn move_smoothly(
    pwm: &mut  Pca9685<I2cDriver<'_>>,
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


fn run_c5_power_test(pwm: &mut Pca9685<RefCellDevice<'_, I2cDriver<'_>>>, current_angle: &mut u32, target_angle: u32) {
    let step_delay = Duration::from_millis(20); // 안정성을 위해 20ms 간격
    
    while *current_angle != target_angle {
        if *current_angle < target_angle {
            *current_angle += 1;
        } else {
            *current_angle -= 1;
        }
        
        // 서보에 PWM 신호 전송
        //set_servo_pwm(*current_angle); 
        pwm.set_channel_on_off(Channel::C5, 0, *current_angle as u16).unwrap();
        //move_5axis_organic(pwm, [250, 300, 300, 350, target_c5], currents);
        
        // 속도보다 '안정성'을 택한 딜레이
        FreeRtos::delay_ms(step_delay.as_millis() as u32);
    }
}