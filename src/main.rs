use std::cell::RefCell;

use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use anyhow::Result;
use esp_idf_hal::units::Hertz;
use esp_idf_svc::http::status::OK;
use pwm_pca9685::{Address, Pca9685,Channel};
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
        (0, "베이스", Channel::C0),
        (1, "어깨",   Channel::C1),
        (2, "팔꿈치", Channel::C2),
        (3, "손목/칼날", Channel::C3),
    ];

    // 초기 위치 저장용 변수
    let mut current_positions = [300u16; 4]; // 0~3번 관절 초기값

    log::info!("=== 관절로봇 테스트 시작 ===");
    loop {
        for (id, name, channel) in channels.iter() {
            println!("🔔 {}번 {} 모터 작동 테스트", id, name);

            let idx = *id as usize;

            match *id {
                0 => { // 베이스: 시원하게 회전
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 150);
                    FreeRtos::delay_ms(1200);
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 450);
                    FreeRtos::delay_ms(1000);
                },
                1 => { // 어깨: 너무 구부리지 않게 범위 축소 (80도 ~ 110도)
                    println!("   -> 어깨는 조심조심 (안전 범위)");
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 280);
                    FreeRtos::delay_ms(1200);
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 330);
                    FreeRtos::delay_ms(1000);
                },
                2 => { // 팔꿈치: 새로 추가된 관절 테스트
                    println!("   -> 팔꿈치 처음으로 움직입니다!");
                    // 350 위치로 부드럽게 이동
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 350);
                    FreeRtos::delay_ms(1000);

                    // 250 위치로 부드럽게 이동
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 250);
                    FreeRtos::delay_ms(1000);
                },
                3 => { // 3. 팔꿈치: 새로 추가된 관절 테스트
                   // 1. 최소 안전 각도 (예: 팔을 가볍게 굽힘)
                    println!("   -> 굽히기 (Safe Zone)");
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 260);
                    FreeRtos::delay_ms(1500);

                    // 2. 최대 안전 각도 (예: 팔을 가볍게 폄)
                    println!("   -> 펴기 (Safe Zone)");
                    move_arm_smooth(&mut pwm, *channel, &mut current_positions[idx], 340);
                    FreeRtos::delay_ms(1500);

                },
                _ => {}
            }
            
            // 다음 모터 테스트 전 중립(300)으로 복귀하여 안정성 확보
            pwm.set_channel_on_off(*channel, 0, 300).unwrap();
            FreeRtos::delay_ms(1000);
        }

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

    let steps = 20; // 분할 단계 (클수록 더 부드러움)
    
    for i in 1..=steps {
        // Sine 함수를 이용한 부드러운 보간 (0.0 ~ 1.0)
        let t = i as f32 / steps as f32;
        let ease_step = (1.0 - (t * std::f32::consts::PI).cos()) / 2.0;
        
        let next_pos = (*current_pos as f32 + (target_pos as i32 - *current_pos as i32) as f32 * ease_step) as u16;
        
        pwm.set_channel_on_off(channel, 0, next_pos).unwrap();
        
        // 이동 중 물리적 부하 분산을 위한 짧은 대기 [cite: 2026-02-13]
        FreeRtos::delay_ms(20); 
    }
    
    *current_pos = target_pos;
}