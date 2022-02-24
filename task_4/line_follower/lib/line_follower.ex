	defmodule LineFollower do
  @moduledoc """
  Documentation for `FW_DEMO`.

  Different functions provided for testing components of Alpha Bot.
  test_wlf_sensors  - to test white line sensors
  test_ir           - to test IR proximity sensors
  test_motion       - to test motion of the Robot
  test_pwm          - to test speed of the Robot
  test_servo_a      - to test servo motor A
  test_servo_b      - to test servo motor B
  """

  require Logger
  use Bitwise
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dr: 16, dl: 19]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @servo_a_pin 27
  @servo_b_pin 22

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @forward [0, 1, 1, 0]
  @left [0, 1, 0, 1]
  @right [1, 0, 1, 0]
  @backward [1, 0, 0, 1]
  @stop [0, 0, 0, 0]
  @sleft [0, 1, 0, 0]
  @sright [0, 0, 1, 0]

  @duty_cycles [150, 70, 0]
  @pwm_frequency 50

  # def wlf do
  #   Logger.debug("Testing white line sensors connected ")
  #   sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
  #   sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
  #   sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
  #   go_to_node([1,2,3,4,5], sensor_ref)
  # end

  #def go_forward do
  #  motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
  #  pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
  #  Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
   # motion_list = [@forward,@stop,@backward,@stop,@right,@stop,@left,@stop]
   # Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion)
   # Process.sleep(1000) end)
  #end


  def main do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
    sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
    sensor_ref = Enum.zip(@ref_atoms, sensor_ref)       
    move(motor_ref, sensor_ref, 0)
    turn(motor_ref, sensor_ref, "right", 0)
    move(motor_ref, sensor_ref, 0)
    move(motor_ref, sensor_ref, 0)
 end

 def move_n(x, n, motor_ref, sensor_ref) do
    if n != x do
      move(motor_ref, sensor_ref, 0)
      n = n +  1
      move_n(x, n, motor_ref, sensor_ref)   
    end
 end

 def turn(motor_ref, sensor_ref, side, state) do
    append_sensor_list = [0,1,2,3,4] ++ [5]
    temp_sensor_list = [5 | append_sensor_list]
    l = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    ls = Enum.at(l, 4)
    cs = Enum.at(l, 3)
    rs = Enum.at(l, 2)
    rrs = Enum.at(l, 1)
    lls = Enum.at(l, 0)
    
    if state == 0 do
     if side == "right" do
      motor_action(motor_ref, @sright)
      motion_pwm(90)
     else
      motor_action(motor_ref, @sleft)
      motion_pwm(90)
     end      
     Process.sleep(300)
     turn(motor_ref, sensor_ref, side, 1)
    else
     if cs>700 do
     motor_action(motor_ref, @stop)
     else
      if side == "right" do
        motor_action(motor_ref, @sright)
        motion_pwm(90)
      else
        motor_action(motor_ref, @sleft)
        motion_pwm(90)
      end
      turn(motor_ref, sensor_ref, side, 1)
     end
    end

 end

 def move(motor_ref, sensor_ref, state) do
    append_sensor_list = [0,1,2,3,4] ++ [5]
    temp_sensor_list = [5 | append_sensor_list]
    l = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    ls = Enum.at(l, 4)
    cs = Enum.at(l, 3)
    rs = Enum.at(l, 2)
    rrs = Enum.at(l, 1) 
    lls = Enum.at(l, 0)
    
    if (((lls>700)&&(ls>700)&&(cs>700)) ||  ((rrs>700)&&(rs>700)&&(cs>700))) && state == 0 do
      motor_action(motor_ref, @forward)
      motion_pwm(110)
      move(motor_ref, sensor_ref, 0)
    else
    cond do
    ((lls>700)&&(ls>700)&&(cs>700)) ||  ((rrs>700)&&(rs>700)&&(cs>700)) -> motor_action(motor_ref, @stop)             
    cs<700 -> if (ls>700 || lls>700) do
		motor_action(motor_ref, @sright)
	        motion_pwm(100)
                move(motor_ref, sensor_ref, 1)
              else
		motor_action(motor_ref, @sleft)
	        motion_pwm(100)
                move(motor_ref, sensor_ref, 1)
              end
    true ->
       motor_action(motor_ref, @forward)
       motion_pwm(150)
       move(motor_ref, sensor_ref, 1) 
    end
    end
 end
  
  
  defp configure_sensor({atom, pin_no}) do
    if (atom == :dataout) do
      GPIO.open(pin_no, :input, pull_mode: :pullup)
    else
      GPIO.open(pin_no, :output)
    end
  end

  defp analog_read(sens_num, sensor_ref, {_, sensor_atom_num}) do

    GPIO.write(sensor_ref[:cs], 0)
    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    Enum.reduce(0..9, @lf_sensor_data, fn n, acc ->
        read_data(n, acc, sens_num, sensor_ref, sensor_atom_num)
        |> clock_signal(n, sensor_ref)
      end)[sensor_atom]
  end

  defp provide_clock(sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
  end

  defp read_data(n, acc, sens_num, sensor_ref, sensor_atom_num) do
    if (n < 4) do

      if (((sens_num) >>> (3 - n)) &&& 0x01) == 1 do
        GPIO.write(sensor_ref[:address], 1)
      else
        GPIO.write(sensor_ref[:address], 0)
      end
      Process.sleep(1)
    end

    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    if (n <= 9) do
      Map.update!(acc, sensor_atom, fn sensor_atom -> ( sensor_atom <<< 1 ||| GPIO.read(sensor_ref[:dataout]) ) end)
    end
  end

  defp clock_signal(acc, n, sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
    acc
  end

  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
  end

  defp motion_pwm(value) do
    pwm(value)
  end

  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end
