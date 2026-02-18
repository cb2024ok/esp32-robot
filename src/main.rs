use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use anyhow::Result;
use esp_idf_hal::units::Hertz;
use pwm_pca9685::{Address, Pca9685};

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

    let mut pwm = Pca9685::new(i2c, Address::from(0x60))
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

    cmd.set_high()?;
    clk.set_high()?;
    att.set_high()?;

    // 이전 상태 저장용 변수 (버튼 2개 + 스틱 4개 = 총 6개)
    let mut last_data = [0u8; 6]; 

    println!("🚀 [이벤트 모드] 버튼을 누르거나 스틱을 움직일 때만 로그가 찍힙니다!");

    loop {
        att.set_low()?;
        Ets::delay_us(15);

        let mut current_data = [0u8; 6];
        let commands = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

        // 9바이트를 읽어야 스틱 4개(RX, RY, LX, LY) 데이터를 다 가져옵니다.
        let mut full_response = [0u8; 9];
        for i in 0..9 {
            let mut byte = 0u8;
            for bit in 0..8 {
                if (commands[i] & (1 << bit)) != 0 { cmd.set_high()?; } else { cmd.set_low()?; }
                clk.set_low()?;
                Ets::delay_us(10);
                if dat.get_level() == Level::High { byte |= 1 << bit; }
                clk.set_high()?;
                Ets::delay_us(10);
            }
            full_response[i] = byte;
        }
        att.set_high()?;

        // 실제 유의미한 데이터 추출
        // [버튼1, 버튼2, RX, RY, LX, LY]
        let current_payload = [
            full_response[3], full_response[4], 
            full_response[5], full_response[6], 
            full_response[7], full_response[8]
        ];

        // [비밀] 데이터에 변화가 있을 때만 출력!
        if full_response[2] == 0x5A && current_payload != last_data {

            let b1 = current_payload[0]; // 첫 번째 버튼 바이트 (Select, L3, R3, Start, Up, Right, Down, Left)
            let b2 = current_payload[1]; // 두 번째 버튼 바이트 (L2, R2, L1, R1, △, ○, ❌, □)

           println!("🔔 컨트롤러 입력 감지!");
        
            // --- 버튼 이름 판별 (비트가 0일 때 눌린 것) ---
            print!("👉 누른 버튼: ");
            if b2 & 0x10 == 0 { print!("[△ TRIANGLE] "); }
            if b2 & 0x20 == 0 { print!("[○ CIRCLE] "); }
            if b2 & 0x40 == 0 { print!("[❌ CROSS] "); }
            if b2 & 0x80 == 0 { print!("[□ SQUARE] "); }
            
            if b2 & 0x01 == 0 { print!("[L2] "); }
            if b2 & 0x02 == 0 { print!("[R2] "); }
            if b2 & 0x04 == 0 { print!("[L1] "); }
            if b2 & 0x08 == 0 { print!("[R1] "); }

            if b1 & 0x10 == 0 { print!("[↑ UP] "); }
            if b1 & 0x40 == 0 { print!("[↓ DOWN] "); }
            if b1 & 0x80 == 0 { print!("[← LEFT] "); }
            if b1 & 0x20 == 0 { print!("[→ RIGHT] "); }
            
            if b1 & 0x01 == 0 { print!("[SELECT] "); }
            if b1 & 0x08 == 0 { print!("[START] "); }
            println!(); // 줄바꿈

            // --- 스틱 값 출력 ---
            println!("🕹️ 스틱 L: ({:3}, {:3}) | R: ({:3}, {:3})", 
                    current_payload[4], current_payload[5],  // LX, LY
                    current_payload[2], current_payload[3]); // RX, RY
            println!("------------------------------------"); 

            // 현재 상태를 저장
            last_data = current_payload;
        }

        FreeRtos::delay_ms(20);
    }
}

// 이런 식으로 버튼 비트를 체크하는 함수를 넣으면 좋습니다.
fn print_button_name(b1: u8, b2: u8) {
    if b1 & 0x40 == 0 { println!("🔘 Pressed: Cross (X)"); }
    if b1 & 0x20 == 0 { println!("🔘 Pressed: Circle (○)"); }
    if b1 & 0x80 == 0 { println!("🔘 Pressed: Square (□)"); }
    if b1 & 0x10 == 0 { println!("🔘 Pressed: Triangle (△)"); }
    // ... 나머지 버튼들도 이런 식으로 추가 가능
}