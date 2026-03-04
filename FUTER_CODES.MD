// BLE 특성(Characteristic)에 값이 써질 때 호출되는 콜백
characteristic.on_write(|value| {
    // 1. 수신 데이터 파싱 (예: "5:450")
    if let Ok(cmd) = std::str::from_utf8(value) {
        let parts: Vec<&str> = cmd.split(':').collect();
        if parts.len() == 2 {
            let servo_id = parts[0];
            if let Ok(target) = parts[1].parse::<u32>() {
                if servo_id == "5" { // 그리퍼 채널
                    // 2. 안전 범위 적용 및 타겟 업데이트
                    let safe_target = target.clamp(185, 480);
                    INPUT_TARGET.store(safe_target, Ordering::SeqCst);
                }
            }
        }
    }
});

// 메인 루프에서는 계속 관찰
loop {
    let target = INPUT_TARGET.load(Ordering::SeqCst);
    // 3. 이미 검증된 부드러운 이동 로직 실행
    run_c5_power_test(&mut pwm, &mut current_angle, target);
    FreeRtos::delay_ms(10);
}