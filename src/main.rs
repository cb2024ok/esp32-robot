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


fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    let peripherals = Peripherals::take()?;

    println!("🚀 [테스트] I2C를 제외하고 PS2 컨트롤러만 시작합니다...");

    // --- I2C 및 PCA9685 로직 일시 중단 ---
    let config = I2cConfig::new().baudrate(Hertz(100_000));
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        &config,
    )?;

    // 1. I2C 드라이버를 공유 버스로 감쌉니다.
    //let i2c_bus = shared_bus::BusManagerSimple::new(i2c);

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

    //println!("✅ 모터 드라이버(PCA9685) 연결 성공!");

    // 핀 설정 (이전과 동일)
    let mut dat = PinDriver::input(peripherals.pins.gpio19)?;
    dat.set_pull(Pull::Up)?; 
    let mut cmd = PinDriver::output(peripherals.pins.gpio23)?;
    let mut clk = PinDriver::output(peripherals.pins.gpio18)?;
    let mut att = PinDriver::output(peripherals.pins.gpio5)?;

    //cmd.set_high()?;
    //clk.set_high()?;
    //att.set_high()?;

    // New..
    att.set_low()?;
    Ets::delay_ms(100);

    // Config 모드 진입 명령 (0x01 0x43 0x00 0x01 0x00)
    let mut config_enter = [0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00];
    let mut resp = [0u8; 9];
    for i in 0..9 {
        let byte = if i < config_enter.len() { config_enter[i] } else { 0x00 };
        let mut recv = 0u8;
        for bit in 0..8 {
            if (byte & (1 << bit)) != 0 { cmd.set_high()?; } else { cmd.set_low()?; }
            clk.set_low()?;
            Ets::delay_us(15);  // 15us로 늘려서 안정성 UP
            if dat.get_level() == Level::High { recv |= 1 << bit; }
            clk.set_high()?;
            Ets::delay_us(15);
        }
        resp[i] = recv;
    }
    att.set_high()?;
    println!("Config Enter Resp: {:?}", resp);

    // 아날로그 + Lock 설정 (0x01 0x44 0x00 0x01 0x03 ...)
    let mut analog_on = [0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00];
    att.set_low()?;
    Ets::delay_us(100);
    for i in 0..9 {
        let byte = if i < analog_on.len() { analog_on[i] } else { 0x00 };
        let mut recv = 0u8;
        for bit in 0..8 {
            if (byte & (1 << bit)) != 0 { cmd.set_high()?; } else { cmd.set_low()?; }
            clk.set_low()?;
            Ets::delay_us(15);
            if dat.get_level() == Level::High { recv |= 1 << bit; }
            clk.set_high()?;
            Ets::delay_us(15);
        }
        resp[i] = recv;
    }
    att.set_high()?;
    println!("Analog ON Resp: {:?}", resp);


    // 이전 상태 저장용 변수 (버튼 2개 + 스틱 4개 = 총 6개)
    let mut last_data = [0u8; 6]; 

    println!("🚀 [이벤트 모드] 버튼을 누르거나 스틱을 움직일 때만 로그가 찍힙니다!");

    // 1. ESP32-C6 하드웨어 초기화 (GPIO, PWM 등)
    let mut controller = RobotArmController::new();

    loop {
        att.set_low()?;
        Ets::delay_us(20);

        let mut current_data = [0u8; 8];
        let commands = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

        // 9바이트를 읽어야 스틱 4개(RX, RY, LX, LY) 데이터를 다 가져옵니다.
        let mut full_response = [0u8; 9];
        for i in 0..9 {
            let mut byte = 0u8;
            for bit in 0..8 {
                if (commands[i] & (1 << bit)) != 0 { cmd.set_high()?; } else { cmd.set_low()?; }
                clk.set_low()?;
                Ets::delay_us(15);
                if dat.get_level() == Level::High { byte |= 1 << bit; }
                clk.set_high()?;
                Ets::delay_us(15);
            }
            full_response[i] = byte;
        }
        att.set_high()?;

        //println!(">> Full Response-> {:?}",full_response);

        // 아날로그 모드 확인용
        if full_response[1] == 0x73 || full_response[1] == 0x79 {
            println!("아날로그 모드 OK! LED 빨강일 거야~");
        }

        // 스틱 값 변화 체크 (128에서 벗어나면 성공!)
        let rx = full_response[5];
        let ry = full_response[6];
        let lx = full_response[7];
        let ly = full_response[8];
        /*if rx != 128 || ry != 128 || lx != 128 || ly != 128 {
            println!("스틱 움직임 감지!! RX:{} RY:{} LX:{} LY:{}", rx, ry, lx, ly);
        }
        */

    // 2. PS2 데이터를 로직에서 쓸 수 있게 페이로드 형식으로 변환
    // PS2 패킷 구조: [ID, Mode, Header, 버튼1, 버튼2, RX, RY, LX, LY]
    let ps2_payload = [
        full_response[3], // 버튼 바이트 1
        full_response[4], // 버튼 바이트 2
        full_response[5], // RX (오른쪽 스틱 X)
        full_response[6], // RY
        full_response[7], // LX
        full_response[8], // LY
    ];

        //let mut controller_i2c = RefCellDevice::new(&i2c_ref_cell) ;
        //if let Ok(current_payload) = read_controller_data(&mut controller_i2c)  {

        //println!("I2C Controller Data: {:?}", current_payload); 
        // [비밀] 데이터에 변화가 있을 때만 출력!
        //if full_response[2] == 0x5A && current_payload != last_data {
        if ps2_payload != last_data {

            let b1 =  ps2_payload[0]; //current_payload[0]; // 첫 번째 버튼 바이트 (Select, L3, R3, Start, Up, Right, Down, Left)
            let b2 =  ps2_payload[1]; //current_payload[1]; // 두 번째 버튼 바이트 (L2, R2, L1, R1, △, ○, ❌, □)

           println!("🔔 컨트롤러 입력 감지!");
           println!("B1: {:08b} | B2: {:08b}", b1, b2); // 이진수 출력
        
            // [1] 버튼 제어: R1/L1으로 모터 선택 (그록 비트맵 참고)
            if (b2 &  MASK_R1) == 0 { // R1 클릭
                controller.master_id = (controller.master_id + 1) % 6;
                println!("🎯 마스터 변경 -> #{}", controller.master_id);
                FreeRtos::delay_ms(200); // 중복 클릭 방지
            } else if (b2 & MASK_L1) == 0 { // L1 클릭
                controller.master_id = if controller.master_id == 0 { 5 } else { controller.master_id - 1 };
                println!("🎯 마스터 변경 -> #{}", controller.master_id);
                    FreeRtos::delay_ms(200);
            }


         // 조이스틱 처리 로직
let ly_raw = ps2_payload[5] as i16; // 왼쪽 스틱 상하
let center = 128_i16;
let diff = center - ly_raw; // 위로 밀면 +, 아래로 밀면 -

if diff.abs() > DEADZONE {
    // 1. 방향 결정 (-1.0 ~ 1.0 사이의 비율 계산)
    let direction_ratio = diff as f32 / 128.0;
    
    // 2. 현재 마스터 모터의 각도 가져오기
    let current_angle = controller.angles[controller.master_id];
    
    // 3. 새로운 각도 계산 (현재 각도 + (방향 * 감도))
    // 이 부분이 핵심입니다: diff가 음수면 자동으로 뺄셈이 됩니다.
    let mut next_angle = current_angle + (direction_ratio * SENSITIVITY);
    
    // 4. 안전 범위 제한 (0~180도를 절대 벗어나지 않도록)
    next_angle = next_angle.clamp(MIN_ANGLE, MAX_ANGLE);
    
    // 5. 상태 업데이트
    controller.angles[controller.master_id] = next_angle;

    // 6. PWM 적용
    let duty = angle_to_duty(next_angle);
    let channel = match controller.master_id {
        0 => Channel::C0, 1 => Channel::C1, 2 => Channel::C2,
        3 => Channel::C3, 4 => Channel::C4, 5 => Channel::C5,
        _ => Channel::C0,
    };
    
    let _ = pwm.set_channel_on_off(channel, 0, duty);

    println!("🔄 [MOVE] M:{} | Dir:{:.2} | New Angle:{:.1}", 
              controller.master_id, direction_ratio, next_angle);
} 


            // [4] 특수 기능: △ 버튼 누르면 현재 모든 모터 각도 덤프 (데이터 수집용)
            if (b2 & 0x08) == 0 {
                println!("📝 [RECORD DATA] Current Pose: {:?}", controller.angles);
                FreeRtos::delay_ms(500);
            }

            // 현재 상태를 저장
            last_data = ps2_payload; //current_payload;
        }

        FreeRtos::delay_ms(25);
    }
}

// 예시: 각도(0~180)를 PCA9685 duty 값으로 변환
fn angle_to_duty(angle: f32) -> u16 {
    // 일반적인 서보 모터 기준 (50Hz 기준 1ms ~ 2ms)
    // PCA9685의 12비트(0~4095) 해상도에 맞게 매핑
    let min_duty  = 150.0; // 0도 근처
    let max_duty = 600.0; // 180도 근처
    
    let duty = min_duty + (angle / 180.0) * (max_duty - min_duty);
    duty as u16
}

// 컨트롤러로부터 데이터를 읽어오는 전용 함수
fn read_controller_data<I>(i2c: &mut I) -> Result<[u8; 8]>
    where I: embedded_hal::i2c::I2c
{
    let mut data = [0u8; 8];
    // 컨트롤러의 I2C 주소 (예: 0x20)와 통신
    // 실제 사용하시는 주소로 변경하세요.
    match i2c.read(0x20, &mut data) {
        Ok(_) => Ok(data),
        Err(e) => {
            // 에러 발생 시 로그를 남기고 에러 반환
            // println!("컨트롤러 읽기 에러: {:?}", e);
            Err(anyhow::anyhow!("I2C Read Error: {:?}", e))
        }
    }
}