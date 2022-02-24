defmodule Task4CClientRobotB do
  # max x-coordinate of table top
  @table_top_x 6
  # max y-coordinate of table top
  @table_top_y :f
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> Task4CClientRobotB.place
      {:ok, %Task4CClientRobotB.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %Task4CClientRobotB.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing) when facing not in [:north, :east, :south, :west] do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> Task4CClientRobotB.place(1, :b, :south)
      {:ok, %Task4CClientRobotB.Position{facing: :south, x: 1, y: :b}}

      iex> Task4CClientRobotB.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> Task4CClientRobotB.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %Task4CClientRobotB.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    place(x, y, facing)
  end

  @doc """
  Main function to initiate the sequence of tasks to achieve by the Client Robot B,
  such as connect to the Phoenix server, get the robot B's start and goal locations to be traversed.
  Call the respective functions from this module and others as needed.
  You may create extra helper functions as needed.
  """
  def main do

    ###########################
    ## complete this funcion ##
    ###########################
    [ch, ch2] = Task4CClientRobotB.PhoenixSocketClient.connect_server()
    goal_locs = Task4CClientRobotB.PhoenixSocketClient.get_goals(ch)
    start_pos = Task4CClientRobotB.PhoenixSocketClient.get_start_pos(ch) 
    {:ok, robot} = start(Enum.at(start_pos, 0) |> String.to_integer(), Enum.at(start_pos, 1) |> String.to_atom(), Enum.at(start_pos, 2) |> String.to_atom())
    gl = Enum.map(goal_locs, fn g -> g["pos"] end)
    stop(robot, gl, ch, ch2)


  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Make a call to ToyRobot.PhoenixSocketClient.send_robot_status/2 to get the indication of obstacle presence ahead of the robot.
  """

  def stop(robot, goal_locs, ch, ch2) do

    ###########################
    ## complete this funcion ##
    ###########################
  current_process = self()
    pid_client = spawn_link(fn ->
      robot_tuple = 
      if length(goal_locs) == 1 do
        robot = efficient_reach(robot, goal_locs, ch, ch2)
        {:ok, robot}
      else
        reach_all_goals(robot, goal_locs, ch, ch2)
      end
      send(current_process, robot_tuple)
    end)
    receive do
      {:ok, robot} ->
        {:ok, robot}
    end
  end

  def efficient_reach(robot, goal_locs, cli_proc_name, ch2) do
    gx = Enum.at(goal_locs, 0) |> Enum.at(0) |> String.to_integer()
    gy = Enum.at(goal_locs, 0) |> Enum.at(1) |> String.to_atom()
    n = temp_reach_goal(robot, gx, gy, 0)
    Process.sleep(500)
    # Task4CClientRobotB.PhoenixSocketClient.send_to_a(ch2, {:toyrobotA_n, n})
    receive do
      {:toyrobotB_n, n2} ->
        cond do
          n <= n2 -> 
            robot = reach_goal(robot, gx, gy, cli_proc_name, ch2, [:straight])
            {:obstacle_presence, obs} = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
            # Task4CClientRobotB.PhoenixSocketClient.send_to_a(ch2, {:toyrobotA_er_over, true})
            robot
          true -> 
            {:obstacle_presence, obs} = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
            receive do
              {:toyrobotB_er_over, done} ->
                nil
            end
            robot
        end
    end
  end

  def reach_all_goals(robot, goal_locs, cli_proc_name, ch2) do
    goal_locs = Task4CClientRobotB.PhoenixSocketClient.get_updated_goals(cli_proc_name)
    if Enum.empty?(goal_locs) do
      {:obstacle_presence, obs} = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
      Task4CClientRobotB.PhoenixSocketClient.stop_process(cli_proc_name)
      {:ok, robot}
    else
      [gx, gy] = find_closest(robot, goal_locs)
      Task4CClientRobotB.PhoenixSocketClient.update_goals(cli_proc_name, [gx, gy])
      robot = reach_goal(robot, gx, gy, cli_proc_name, ch2, [:straight])
      reach_all_goals(robot, goal_locs, cli_proc_name, ch2)
    end
  end

  def find_closest(robot, goal_locs) do
    glist = Enum.map(goal_locs, fn g -> [g |> Enum.at(0) |> String.to_integer(), g |> Enum.at(1) |> String.to_atom()] end)
    c = Enum.map(glist, fn g ->
      [temp_reach_goal(robot, Enum.at(g,0), Enum.at(g,1), 0), Enum.find_index(glist, fn g2 -> g2 == g end)] end)
    sort_c = Enum.sort(c)
    close_goal = Enum.at(glist, hd(sort_c) |> Enum.at(1))
	  close_goal
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

  def check_for_robot(robot, ch) do
    pos = Task4CClientRobotB.PhoenixSocketClient.get_a_pos(ch)
    tr = move(robot)
    if tr.x == pos["x"] && Atom.to_string(tr.y) == pos["y"] do
      true
    else
      false
    end
  end

  def reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)do
    # stop 4,c | 3,e | 3,b | 1,c  start 1,c,east | 3,e,south  stop 4,c | 3,b
    Process.sleep(200)
    if robot.x == gx && robot.y == gy do
      robot
    else    
    {:obstacle_presence, obs} = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
    t_robot = move(robot)
    cond do
      check_for_robot(robot, ch2) == [t_robot.x, t_robot.y] && (closer_to_goal(move(robot), {:x, gx}) || closer_to_goal(move(robot), {:y, gy})) ->
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      (closer_to_goal(robot, {:x, gx}) || closer_to_goal(robot, {:y, gy})) && obs == false ->
        robot = move(robot)
        face_list = [:straight]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      Enum.member?(face_list, :right) ->
        if obs do
          robot = right(robot)
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = move(robot)
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        end

      Enum.member?(face_list, :left) ->
        if obs do
          robot = left(robot)
          face_list = face_list ++ [:left]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = move(robot)
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        end
      closer_to_goal(right(robot), {:x, gx}) || closer_to_goal(right(robot), {:y, gy}) ->
        robot = right(robot)
        face_list = face_list ++ [:right]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      closer_to_goal(left(robot), {:x, gx}) || closer_to_goal(left(robot), {:y, gy}) ->
        robot = left(robot)
        face_list = face_list ++ [:left]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      true ->
        if move(left(robot)) == left(robot) do
          robot = right(robot)
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = left(robot)
          face_list = face_list ++ [:left]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
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
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = Task4CClientRobotB.place(2, :b, :west)
      iex> Task4CClientRobotB.report(robot)
      {2, :b, :west}
  """
  def report(%Task4CClientRobotB.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%Task4CClientRobotB.Position{facing: facing} = robot) do
    %Task4CClientRobotB.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%Task4CClientRobotB.Position{facing: facing} = robot) do
    %Task4CClientRobotB.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %Task4CClientRobotB.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
    }
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %Task4CClientRobotB.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %Task4CClientRobotB.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %Task4CClientRobotB.Position{robot | x: x - 1}
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
