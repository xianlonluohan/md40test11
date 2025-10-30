// 在setup()中，将所有字符串常量移到Flash中
void setup() {
  Serial.begin(115200);
  Wire.begin();
  g_md40.Init();

  // 使用F()宏将字符串常量存储在Flash中而不是RAM中
  Serial.print(F("Device ID: 0x"));
  Serial.println(g_md40.device_id(), HEX);
  Serial.print(F("Name: "));
  Serial.println(g_md40.name());
  Serial.print(F("Firmware Version: "));
  Serial.println(g_md40.firmware_version());

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, em::Md40::Motor::PhaseRelation::kAPhaseLeads);
    g_md40[i].set_speed_pid_p(1.5);
    g_md40[i].set_speed_pid_i(1.5);
    g_md40[i].set_speed_pid_d(1.0);
    g_md40[i].set_position_pid_p(10.0);
    g_md40[i].set_position_pid_i(1.0);
    g_md40[i].set_position_pid_d(1.0);

    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.print(F(" state:"));
    Serial.print(static_cast<uint8_t>(g_md40[i].state()));
    Serial.print(F(", speed pid p:"));
    Serial.print(g_md40[i].speed_pid_p());
    // ... 其他Serial.print也加上F()宏
  }

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.print(F(" run speed: "));
    Serial.println(kMotorPwmDuty);
    g_md40[i].RunPwmDuty(kMotorPwmDuty);
  }
}

void loop() {
  if (g_last_print_time == 0 || millis() - g_last_print_time > 200) {
    g_last_print_time = millis();
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      const auto speed = g_md40[i].speed();

      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" speed: "));
      Serial.print(speed);

      if (speed > 0) {
        Serial.println(F(". The phase of A leads B, constructed with the em::Md40::Motor::PhaseRelation::kAPhaseLeads enum."));
      } else if (speed < 0) {
        Serial.println(F(". The phase of B leads A, constructed with the em::Md40::Motor::PhaseRelation::kBPhaseLeads enum."));
      } else {
        Serial.println(F(". The motor is not running currently."));
      }
    }
    Serial.println();
  }
}