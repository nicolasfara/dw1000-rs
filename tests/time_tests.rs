use dw1000_rs::time::*;
use dw1000_rs::DW1000Time;

#[test]
fn test_new() {
    let time = DW1000Time::new();
    assert_eq!(time.get_timestamp(), 0);
}

#[test]
fn test_from_timestamp() {
    let time = DW1000Time::from_timestamp(1000);
    assert_eq!(time.get_timestamp(), 1000);
}

#[test]
fn test_from_bytes() {
    let data: [u8; 5] = [0x01, 0x02, 0x03, 0x04, 0x05];
    let time = DW1000Time::from_bytes(&data);
    let expected: i64 = 0x01 | (0x02 << 8) | (0x03 << 16) | (0x04 << 24) | (0x05 << 32);
    assert_eq!(time.get_timestamp(), expected);
}

#[test]
fn test_to_bytes() {
    let time = DW1000Time::from_timestamp(0x0504030201);
    let bytes = time.to_bytes();
    assert_eq!(bytes, [0x01, 0x02, 0x03, 0x04, 0x05]);
}

#[test]
fn test_add() {
    let time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(50);
    let result = time1 + time2;
    assert_eq!(result.get_timestamp(), 150);
}

#[test]
fn test_add_assign() {
    let mut time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(50);
    time1 += time2;
    assert_eq!(time1.get_timestamp(), 150);
}

#[test]
fn test_sub() {
    let time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(50);
    let result = time1 - time2;
    assert_eq!(result.get_timestamp(), 50);
}

#[test]
fn test_sub_assign() {
    let mut time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(50);
    time1 -= time2;
    assert_eq!(time1.get_timestamp(), 50);
}

#[test]
fn test_mul_f32() {
    let time = DW1000Time::from_timestamp(100);
    let result = time * 2.0;
    assert_eq!(result.get_timestamp(), 200);
}

#[test]
fn test_mul_assign_f32() {
    let mut time = DW1000Time::from_timestamp(100);
    time *= 2.0;
    assert_eq!(time.get_timestamp(), 200);
}

#[test]
fn test_mul_dw1000time() {
    let time1 = DW1000Time::from_timestamp(10);
    let time2 = DW1000Time::from_timestamp(5);
    let result = time1 * time2;
    assert_eq!(result.get_timestamp(), 50);
}

#[test]
fn test_div_f32() {
    let time = DW1000Time::from_timestamp(100);
    let result = time / 2.0;
    assert_eq!(result.get_timestamp(), 50);
}

#[test]
fn test_div_assign_f32() {
    let mut time = DW1000Time::from_timestamp(100);
    time /= 2.0;
    assert_eq!(time.get_timestamp(), 50);
}

#[test]
fn test_div_dw1000time() {
    let time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(5);
    let result = time1 / time2;
    assert_eq!(result.get_timestamp(), 20);
}

#[test]
fn test_wrap_negative() {
    let mut time = DW1000Time::from_timestamp(-100);
    time.wrap();
    assert_eq!(time.get_timestamp(), TIME_OVERFLOW - 100);
}

#[test]
fn test_wrap_positive() {
    let mut time = DW1000Time::from_timestamp(100);
    time.wrap();
    assert_eq!(time.get_timestamp(), 100);
}

#[test]
fn test_wrapped() {
    let time = DW1000Time::from_timestamp(-100);
    let wrapped = time.wrapped();
    assert_eq!(wrapped.get_timestamp(), TIME_OVERFLOW - 100);
    // Original should be unchanged
    assert_eq!(time.get_timestamp(), -100);
}

#[test]
fn test_is_valid_timestamp() {
    let time1 = DW1000Time::from_timestamp(100);
    assert!(time1.is_valid_timestamp());

    let time2 = DW1000Time::from_timestamp(-100);
    assert!(!time2.is_valid_timestamp());

    let time3 = DW1000Time::from_timestamp(TIME_MAX + 1);
    assert!(!time3.is_valid_timestamp());

    let time4 = DW1000Time::from_timestamp(TIME_MAX);
    assert!(time4.is_valid_timestamp());
}

#[test]
fn test_equality() {
    let time1 = DW1000Time::from_timestamp(100);
    let time2 = DW1000Time::from_timestamp(100);
    let time3 = DW1000Time::from_timestamp(200);

    assert_eq!(time1, time2);
    assert_ne!(time1, time3);
}

#[test]
fn test_as_microseconds() {
    let time = DW1000Time::from_timestamp(TIME_RES_INV as i64);
    let us = time.as_microseconds();
    // Should be approximately 1.0 microsecond
    assert!((us - 1.0).abs() < 0.01);
}

#[test]
fn test_from_microseconds() {
    let time = DW1000Time::from_microseconds(1.0);
    let us = time.as_microseconds();
    // Should round-trip approximately
    assert!((us - 1.0).abs() < 0.01);
}

#[test]
fn test_from_time_with_factor() {
    let time = DW1000Time::from_time_with_factor(1, MILLISECONDS);
    let us = time.as_microseconds();
    // 1 millisecond = 1000 microseconds
    assert!((us - 1000.0).abs() < 1.0);
}

#[test]
fn test_set_and_get_timestamp() {
    let mut time = DW1000Time::new();
    time.set_timestamp(12345);
    assert_eq!(time.get_timestamp(), 12345);
}

#[test]
fn test_set_timestamp_from_bytes() {
    let mut time = DW1000Time::new();
    let data: [u8; 5] = [0xFF, 0xEE, 0xDD, 0xCC, 0xBB];
    time.set_timestamp_from_bytes(&data);

    let mut read_back = [0u8; 5];
    time.get_timestamp_bytes(&mut read_back);
    assert_eq!(data, read_back);
}

#[test]
fn test_set_time() {
    let mut time = DW1000Time::new();
    time.set_time(100.0);
    let us = time.as_microseconds();
    assert!((us - 100.0).abs() < 1.0);
}

#[test]
fn test_set_time_with_factor() {
    let mut time = DW1000Time::new();
    time.set_time_with_factor(5, SECONDS);
    let us = time.as_microseconds();
    // 5 seconds = 5,000,000 microseconds
    assert!((us - 5_000_000.0).abs() < 100.0);
}

#[test]
fn test_as_meters() {
    let time = DW1000Time::from_timestamp(1000);
    let meters = time.as_meters();
    // Just verify it returns a reasonable value
    assert!(meters > 0.0);
}

#[test]
fn test_default() {
    let time = DW1000Time::default();
    assert_eq!(time.get_timestamp(), 0);
}

#[test]
fn test_clone() {
    let time1 = DW1000Time::from_timestamp(12345);
    let time2 = time1.clone();
    assert_eq!(time1, time2);
}

#[test]
fn test_copy() {
    let time1 = DW1000Time::from_timestamp(12345);
    let time2 = time1; // Copy semantics
    assert_eq!(time1, time2);
    // Both should still be usable
    assert_eq!(time1.get_timestamp(), 12345);
    assert_eq!(time2.get_timestamp(), 12345);
}

#[test]
fn test_overflow_scenario() {
    // Simulate the example from the documentation
    let timestamp_sent = DW1000Time::from_timestamp(TIME_MAX - 10);
    let _delay = DW1000Time::from_timestamp(20);

    // This would overflow in real hardware
    let timestamp_received = DW1000Time::from_timestamp(9); // After overflow

    let mut time_diff = timestamp_received - timestamp_sent;
    // Without wrap, this would be negative
    assert!(time_diff.get_timestamp() < 0);

    // After wrapping, should be positive
    time_diff.wrap();
    assert!(time_diff.get_timestamp() > 0);
}
