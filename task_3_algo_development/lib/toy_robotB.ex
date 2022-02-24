defmodule CLI.ToyRobotB do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> CLI.ToyRobotB.place
      {:ok, %CLI.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %CLI.Position{}}
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

      iex> CLI.ToyRobotB.place(1, :b, :south)
      {:ok, %CLI.Position{facing: :south, x: 1, y: :b}}

      iex> CLI.ToyRobotB.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> CLI.ToyRobotB.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    # IO.puts String.upcase("B I'm placed at => #{x},#{y},#{facing}")
    {:ok, %CLI.Position{x: x, y: y, facing: facing}}
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

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobotB' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_locs, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################
    current_process = self()
    pid_client = spawn_link(fn ->
      robot_tuple = 
      if length(goal_locs) == 1 do
        robot = efficient_reach(robot, goal_locs, cli_proc_name)
        {:ok, robot}
      else
        reach_all_goals(robot, goal_locs, cli_proc_name)
      end
      send(current_process, robot_tuple)
    end)
    Process.register(pid_client, :client_toyrobotB)
    receive do
      {:ok, robot} ->
        {:ok, robot}
    end
  end

  def efficient_reach(robot, goal_locs, cli_proc_name) do
    gx = Enum.at(goal_locs, 0) |> Enum.at(0) |> String.to_integer()
    gy = Enum.at(goal_locs, 0) |> Enum.at(1) |> String.to_atom()
    n = temp_reach_goal(robot, gx, gy, 0)
    send(:client_toyrobotA, {:toyrobotB_n, n})
    receive do
      {:toyrobotA_n, n2} ->
        cond do
          n2 <= n -> 
            send_robot_status(robot, cli_proc_name)
            receive do
              {:toyrobotA_er_over, done} ->
                nil
            end
            robot
          true -> 
            robot = reach_goal(robot, gx, gy, cli_proc_name, [:straight])
            send_robot_status(robot, cli_proc_name)
            send(:client_toyrobotA, {:toyrobotB_er_over, true})
            robot
        end
    end
  end

  def reach_all_goals(robot, goal_locs, cli_proc_name) do
    goal_locs = receive do
      {:toyrobotA_goal, gxy} ->
        goal_locs = Enum.filter(goal_locs, fn g -> g != gxy end)
      after
        0 -> goal_locs
    end
    if length(goal_locs) <= 1 do
      send_robot_status(robot, cli_proc_name)
      send(:client_toyrobotA, {:toyrobotB_over, true})
      receive do
        {:toyrobotA_over, done} ->
          nil
      end
      {:ok, robot}
    else
      [[gx, gy], goal_locs] = find_closest(robot, goal_locs)
      send(:client_toyrobotA, {:toyrobotB_goal, [Integer.to_string(gx), Atom.to_string(gy)]})
      robot = reach_goal(robot, gx, gy, cli_proc_name, [:straight])
      reach_all_goals(robot, goal_locs, cli_proc_name)
    end
  end

  def find_closest(robot, goal_locs) do
    glist = Enum.map(goal_locs, fn g -> [g |> Enum.at(0) |> String.to_integer(), g |> Enum.at(1) |> String.to_atom()] end)
    c = Enum.map(glist, fn g ->
      [temp_reach_goal(robot, Enum.at(g,0), Enum.at(g,1), 0), Enum.find_index(glist, fn g2 -> g2 == g end)] end)
    sort_c = Enum.sort(c)
    close_goal = Enum.at(glist, hd(sort_c) |> Enum.at(1))
    gl = List.delete_at(goal_locs, hd(sort_c) |> Enum.at(1))
	  [close_goal, gl]
  end

  def temp_reach_goal(robot, gx, gy, counter) do
    counter = counter + 1
    cond do
      robot.x == gx && robot.y == gy ->
        counter - 1
      closer_to_goal(robot, {:x, gx}) || closer_to_goal(robot, {:y, gy}) ->
        robot = move(robot)
        temp_reach_goal(robot, gx, gy, counter)
      closer_to_goal(right(robot), {:x, gx}) || closer_to_goal(right(robot), {:y, gy}) ->
        robot = right(robot)
        temp_reach_goal(robot, gx, gy, counter)
      true ->
        robot = left(robot)
        temp_reach_goal(robot, gx, gy, counter)
      end
  end

  def check_for_robot(robot) do
    tr = nil
    send(:client_toyrobotA, {:toyrobotB_status, robot})
    tr = receive do
      {:toyrobotA_status, robota} ->
         move(robota)
      after
        0 -> nil
      end
    if tr == nil do
      nil
    else
      [tr.x, tr.y]
    end
  end

  def reach_goal(robot, gx, gy, cli_proc_name, face_list)do
    if robot.x == gx && robot.y == gy do
      robot
    else
    obs = send_robot_status(robot, cli_proc_name)
    t_robot = move(robot)
    cond do
      check_for_robot(robot) == [t_robot.x, t_robot.y] && (closer_to_goal(move(robot), {:x, gx}) || closer_to_goal(move(robot), {:y, gy})) ->
        reach_goal(robot, gx, gy, cli_proc_name, face_list)
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
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%CLI.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobotB_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
    listen_from_server()
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} ->
        is_obs_ahead
    end
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = CLI.ToyRobotB.place(2, :b, :west)
      iex> CLI.ToyRobotB.report(robot)
      {2, :b, :west}
  """
  def report(%CLI.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %CLI.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %CLI.Position{robot | x: x - 1}
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
