use std::{thread, time::Duration};

use esp_idf_hal::{delay::FreeRtos, ledc::LEDC};
use esp_idf_hal::i2c::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use pwm_pca9685::{Address, Channel, Pca9685};

// 전역 상수나 변수로 선언해서 관리하세요
const SHOULDER_MAX_FRONT: u16 = 400; // 이 이상 앞으로 숙이면 넘어짐!
const ELBOW_MIN_LIMIT: u16 = 200;    // 너무 접히면 프레임에 걸림!

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    // 1. 하드웨어 주변장치 가져오기
    let peripherals = Peripherals::take().unwrap();
    
    // 2. I2C 설정 (D1 R32의 SDA: 21, SCL: 22)
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;

    let config = I2cConfig::new().baudrate(10.kHz().into());
    let i2c_driver = I2cDriver::new(i2c, sda, scl, &config.baudrate(10.kHz().into()))?;
    // I2C 설정 부분 수정
/*let i2c_driver= I2cDriver::new(
    peripherals.i2c0,
    peripherals.pins.gpio21, // SDA
    peripherals.pins.gpio22, // SCL
    &I2cConfig::new()
        .baudrate(10.kHz().into()) // 속도를 100kHz -> 10kHz로 낮춤 (안정성 확보)
).map_err(|e| Err(e.to_string()))?;
*/

    // 3. PCA9685 드라이버 초기화 (I2C 주소 0x40)
    //let mut pwm = Pca9685::new(i2c_driver, Address::default()).unwrap();
    let mut pwm = Pca9685::new(i2c_driver, 0x60).map_err(|_| anyhow::anyhow!("PCA9685 초기화 실패"))?;
    pwm.set_prescale(121).unwrap(); // 50Hz 설정 (서보 표준)
    pwm.enable().unwrap();

    println!("🚀 0번 관절(Base) 테스트 시작! 90도로 고정합니다.");
    
    /* 
    // PCA9685 초기화 시도
    let mut pwm = match Pca9685::new(i2c_driver, Address::default()) {
        Ok(mut driver) => {
            println!("✅ PCA9685 연결 성공!");
            driver.set_prescale(121).ok(); 
            driver.enable().ok();
            driver
        },
        Err(e) => {
            println!("❌ PCA9685 찾기 실패: {:?}", e);
            println!("👉 체크리스트: 1.실드 밀착 2.외부5V전원 3.I2C핀 확인");
            // 에러가 나도 죽지 않고 무한 루프에서 대기 (하드웨어 점검 시간 벌기)
            loop { FreeRtos::delay_ms(1000); }
        }
    };
    */
    println!("🎬 1번(C0)과 2번(C1) 모터 동시 테스트 시작!");

    // 초기 위치 설정
    // [중요] 사진 속 'ㄱ'자 자세를 위한 목표 값
    let mut pos0 = 325; // Base (정면)
    let mut pos1 = 325; // Shoulder (초기 수직)
    let mut pos2 = 325; // Elbow (초기 수직)
    let mut pos3 = 325; // Wrist/Gripper (초기 수직)

    // 수정된 안전 타겟 값
let target_pos1 = 300; // 260보다 조금 더 세움 (하중을 뒤로 유지)
let target_pos2 = 380; // 430보다 덜 뻗음 (무게 중심이 베이스 안에 머물도록)

    println!("🏠 기본 자세(ㄱ자) 잡기 시작...");

   // 순서 변경: 어깨를 더 세운 뒤에 팔꿈치를 아주 조금만 뻗습니다.
move_smoothly(&mut pwm, Channel::C1, &mut pos1, target_pos1); 
move_smoothly(&mut pwm, Channel::C2, &mut pos2, target_pos2); 
    
    // 3. 3번 모터(C3) 수평 유지 (325)
    move_smoothly(&mut pwm, Channel::C3, &mut pos3, 325);

    println!("✅ 기본 자세 유지 중. 이제 물리적 중심을 확인하세요!");

    loop {

        /*  
        // --- 1단계: 두 모터 모두 0도 근처 ---
        println!("📍 Position: 0도");
        pwm.set_channel_on_off(pwm_pca9685::Channel::C0, 0, 150).ok();
        pwm.set_channel_on_off(pwm_pca9685::Channel::C1, 0, 150).ok(); // 2번 모터 (오늘 추가!)
        FreeRtos::delay_ms(2000);

        // --- 2단계: 두 모터 모두 90도 ---
        println!("📍 Position: 90도");
        pwm.set_channel_on_off(pwm_pca9685::Channel::C0, 0, 325).ok();
        pwm.set_channel_on_off(pwm_pca9685::Channel::C1, 0, 325).ok();
        FreeRtos::delay_ms(2000);

        // --- 3단계: 두 모터 모두 180도 ---
        println!("📍 위치: 180도");
        pwm.set_channel_on_off(pwm_pca9685::Channel::C0, 0, 500).ok();
        pwm.set_channel_on_off(pwm_pca9685::Channel::C1, 0, 500).ok();
        FreeRtos::delay_ms(2000);
        */

      // 1번 모터(C0) 0도로 이동
        /* 
        println!("📍 1번 모터: 0도 이동 중...");
        move_smoothly(&mut pwm, Channel::C0, &mut pos0, 150, 20);
        
        // 2번 모터(C1) 0도로 이동
        println!("📍 2번 모터: 0도 이동 중...");
        move_smoothly(&mut pwm, Channel::C1, &mut pos1, 150, 20);
        
        FreeRtos::delay_ms(1000);
        */

        // 다시 90도로 복귀
        println!("📍 모든 모터 90도로 복귀 중...");
        
        FreeRtos::delay_ms(1000); 

    }
}

/// target_pos: 목표 펄스 값
// 부드러운 이동 함수 (20ms의 안전 지연)
fn move_smoothly(pwm: &mut Pca9685<I2cDriver>, channel: Channel, current: &mut u16, target: u16) {
    while *current != target {
        if *current < target { *current += 1; } else { *current -= 1; }
        let _ = pwm.set_channel_on_off(channel, 0, *current);
        FreeRtos::delay_ms(20); // 오늘은 이 속도가 생명줄입니다. ㅋ
    }
}