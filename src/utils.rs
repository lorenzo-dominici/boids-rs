pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}