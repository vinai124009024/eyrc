defmodule ArmMechanismTest do
 

  require Logger
  use Bitwise
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dr: 16, dl: 19]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @servo_a_pin 27
  @servo_b_pin 22
  @servo_c_pin 18

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @forward [0, 1, 1, 0]
  @backward [0, 1, 0, 1]
  @left [0, 1, 1, 0]
  @right [1, 0, 0, 1]
  @stop [0, 0, 0, 0]

  @duty_cycles [150, 70, 0]
  @pwm_frequency 50


 def test_servo_a(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_a_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_a_pin, val)
  end

  def test_servo_b(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)
  end
  
  def test_servo_c(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_c_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_c_pin, val)
  end

def sowing do
  test_servo_a(60)
  Process.sleep(1000)
  test_servo_c(90)
  Process.sleep(1000)
  test_servo_a(0)
end

def weeding do
  motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
  pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
  Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
  test_servo_c(90)
  Process.sleep(1000)
  test_servo_a(60)
  Process.sleep(1000)
  test_servo_c(140)
  Process.sleep(1000)
  test_servo_a(0)
  Process.sleep(1000)
  motor_action(motor_ref, @forward)
  motion_pwm(120)
  Process.sleep(1500)
  motor_action(motor_ref, @stop)
  Process.sleep(1000)
  test_servo_a(60)
  Process.sleep(1000)
  test_servo_c(90)
  Process.sleep(1000)
  test_servo_a(0)
end

def test_motion do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
  end

  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    Process.sleep(500)
  end
  
  defp motion_pwm(value) do
    pwm(value)
  end
  
  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end
