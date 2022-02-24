defmodule ToyRobot do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> ToyRobot.place
      {:ok, %ToyRobot.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %ToyRobot.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing)
  when facing not in [:north, :east, :south, :west]
  do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> ToyRobot.place(1, :b, :south)
      {:ok, %ToyRobot.Position{facing: :south, x: 1, y: :b}}

      iex> ToyRobot.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> ToyRobot.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %ToyRobot.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    ###########################
    ## complete this funcion ##
    ###########################
    place(x, y, facing)
  end

  def stop(_robot, goal_x, goal_y, _channel) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the channel PID on the Phoenix Server that will be used to send robot's current status after each action is taken.
  Make a call to ToyRobot.PhoenixSocketClient.send_robot_status/2
  to get the indication of obstacle presence ahead of the robot.
  """
  def stop(robot, goal_x, goal_y, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    current_process = self()
    pid_client = spawn_link(fn ->
      robot_tuple = reach_goal(robot, goal_x, goal_y, cli_proc_name, [:straight])
      send(current_process, robot_tuple)
    end)
    Process.register(pid_client, :client_toyrobot)
    receive do
      {:ok, robot} ->
        {:ok, robot}
    end

    ###########################
  end

  def reach_goal(robot, gx, gy, cli_proc_name, face_list)do
    {:obstacle_presence, obs} = ToyRobot.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
    cond do
      robot.x == gx && robot.y == gy ->
        {:ok, robot}
      (closer_to_goal(robot, {:x, gx}) || closer_to_goal(robot, {:y, gy})) && obs == false ->
        robot = move(robot)
        face_list = [:straight]
        reach_goal(robot, gx, gy, cli_proc_name, face_list)
      Enum.member?(face_list, :right) ->
        if obs do
          robot = right(robot)
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        else
          robot = move(robot)
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        end

      Enum.member?(face_list, :left) ->
        if obs do
          robot = left(robot)
          face_list = face_list ++ [:left]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        else
          robot = move(robot)
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        end
      closer_to_goal(right(robot), {:x, gx}) || closer_to_goal(right(robot), {:y, gy}) ->
        robot = right(robot)
        face_list = face_list ++ [:right]
        reach_goal(robot, gx, gy, cli_proc_name, face_list)
      closer_to_goal(left(robot), {:x, gx}) || closer_to_goal(left(robot), {:y, gy}) ->
        robot = left(robot)
        face_list = face_list ++ [:left]
        reach_goal(robot, gx, gy, cli_proc_name, face_list)
      true ->
        if move(left(robot)) == left(robot) do
          robot = right(robot)
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        else
          robot = left(robot)
          face_list = face_list ++ [:left]
          reach_goal(robot, gx, gy, cli_proc_name, face_list)
        end
      end
  end

  def closer_to_goal(robot, {axis, value})do
    robot_value = Map.get(robot, axis)
    temp_robot_value = Map.get(move(robot), axis)
    cond do
      robot_value < value && robot_value < temp_robot_value ->
        true
      robot_value > value && robot_value > temp_robot_value ->
        true
      true ->
        false
    end
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = ToyRobot.place(2, :b, :west)
      iex> ToyRobot.report(robot)
      {2, :b, :west}
  """
  def report(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %ToyRobot.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %ToyRobot.Position{robot | x: x - 1}
  end

  @doc """
  Does not change the position of the robot.
  This function used as fallback if the robot cannot move outside the table
  """
  def move(robot), do: robot

  def failure do
    raise "Connection has been lost"
  end
end