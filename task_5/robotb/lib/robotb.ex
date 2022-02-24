defmodule Robotb do
  # max x-coordinate of table top
  @table_top_x 6
  # max y-coordinate of table top
  @table_top_y :f
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> Robotb.place
      {:ok, %Robotb.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %Robotb.Position{}}
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

      iex> Robotb.place(1, :b, :south)
      {:ok, %Robotb.Position{facing: :south, x: 1, y: :b}}

      iex> Robotb.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> Robotb.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %Robotb.Position{x: x, y: y, facing: facing}}
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
    [ch, ch2] = Robotb.PhoenixSocketClient.connect_server()
    goal_locs = Robotb.PhoenixSocketClient.get_goals(ch)
    Agent.start_link(fn -> goal_locs end, name: :gl)
    start_pos = Robotb.PhoenixSocketClient.get_start_pos(ch) 
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
    # Robotb.PhoenixSocketClient.send_to_a(ch2, {:toyRobotb_n, n})
    receive do
      {:toyrobotB_n, n2} ->
        cond do
          n <= n2 -> 
            robot = reach_goal(robot, gx, gy, cli_proc_name, ch2, [:straight])
            {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
            # Robotb.PhoenixSocketClient.send_to_a(ch2, {:toyRobotb_er_over, true})
            robot
          true -> 
            {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
            receive do
              {:toyrobotB_er_over, done} ->
                nil
            end
            robot
        end
    end
  end

  def reach_all_goals(robot, goal_locs, cli_proc_name, ch2) do
    goal_locs = Robotb.PhoenixSocketClient.get_updated_goals(cli_proc_name)
    if Enum.empty?(goal_locs) do
      {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
      Robotb.PhoenixSocketClient.stop_process(cli_proc_name)
      {:ok, robot}
    else
      [gx, gy] = find_closest(robot, goal_locs)
      Robotb.PhoenixSocketClient.update_goals(cli_proc_name, [gx, gy])
      robot = reach_goal(robot, gx, gy, cli_proc_name, ch2, [:straight])
      g = Enum.filter(Agent.get(:gl, fn l -> l end), fn l -> l["pos"] == [Integer.to_string(gx), Atom.to_string(gy)] end)
      robot = if Enum.at(g, 0)["task"] == "sowing" do
       [robot, rl] = adjust(robot, cli_proc_name)
        if rl == "r" do
          #Actions.main("sowr")
        else
          #Actions.main("sowl")
        end
       Process.sleep(4000)
       Robotb.PhoenixSocketClient.send_for_eval(3, cli_proc_name, Enum.at(g, 0)["num"] |> String.to_integer())
       robot
      else
       [robot, rl] = adjust(robot, cli_proc_name)
        if rl == "r" do
          #Actions.main("weedr")
        else
          #Actions.main("weedl")
        end
       Process.sleep(4000)
       Robotb.PhoenixSocketClient.send_for_eval(4, cli_proc_name, Enum.at(g, 0)["num"] |> String.to_integer())
       robot = reach_depo(robot, cli_proc_name,ch2)
       Robotb.PhoenixSocketClient.send_for_eval(5, cli_proc_name, [Enum.at(g, 0)["num"] |> String.to_integer()])
       robot
      end
      reach_all_goals(robot, goal_locs, cli_proc_name, ch2)
    end
  end

  def adjust(robot, ch) do
    cond do
      robot.facing == :north -> 
                            {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                            if obs == true do                             
                              robot = right(robot)
                              Robotb.Actions.main("right")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              robot = move(robot)
                              Robotb.Actions.main("move")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              [robot, "l"]
                            else
                              robot = move(robot)
                              Robotb.Actions.main("move")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              [robot, "r"]
                            end

      robot.facing == :west -> [robot, "r"]

      robot.facing == :east -> {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                            if obs == true do                             
                              robot = left(robot)
                              Robotb.Actions.main("left")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              robot = move(robot)
                              Robotb.Actions.main("move")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              [robot, "r"]
                            else
                              robot = move(robot)
                              Robotb.Actions.main("move")
                              Robotb.PhoenixSocketClient.send_robot_status(ch, robot)
                              [robot, "l"]
                            end

      robot.facing == :south -> [robot, "l"]

      true -> nil
    end
  end

  def reach_depo(robot, cli_proc_name,ch2) do
    rx = robot.x
    ry = Map.get(@robot_map_y_atom_to_num, robot.y)
    if 6 - rx < 6 - ry do
      robot = reach_goal(robot, 6, robot.y, cli_proc_name, ch2, [:straight])
      cond do 
        robot.facing == :south -> Robotb.Actions.main("depositl")
                                  robot
        robot.facing == :north -> Robotb.Actions.main("depositr")
                                  robot
        robot.facing == :east ->  robot = left(robot)
                                  Robotb.Actions.main("left")
                                  Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
                                  Robotb.Actions.main("depositr")
                                  robot
        true -> nil
      end
    else
      robot = reach_goal(robot, robot.x, :f, cli_proc_name, ch2, [:straight])
      if robot.x == 1 && robot.facing == :north do
          robot = right(robot)
          Robotb.Actions.main("right")
          Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
          Robotb.Actions.main("depositl")
          robot
      else
        cond do 
          robot.facing == :east -> Robotb.Actions.main("depositl")
                                    robot
          robot.facing == :west -> Robotb.Actions.main("depositr")
                                    robot
          robot.facing == :north ->  robot = left(robot)
                                    Robotb.Actions.main("left")
                                    Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
                                    Robotb.Actions.main("depositr")
                                    robot
          true -> nil
        end
      end
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
    pos = Robotb.PhoenixSocketClient.get_a_pos(ch)
    tr = move(robot)
    if tr.x == pos["x"] && Atom.to_string(tr.y) == pos["y"] do
      true
    else
      false
    end
  end

  def reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)do
    # stop 4,c | 3,e | 3,b | 1,c  start 1,c,east | 3,e,south  stop 4,c | 3,b
    Process.sleep(1500)
    if robot.x == gx && robot.y == gy do
     Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
      robot
    else    
    {:obstacle_presence, obs} = Robotb.PhoenixSocketClient.send_robot_status(cli_proc_name, robot)
    t_robot = move(robot)
    cond do
      check_for_robot(robot, ch2) == [t_robot.x, t_robot.y] && (closer_to_goal(move(robot), {:x, gx}) || closer_to_goal(move(robot), {:y, gy})) ->
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      (closer_to_goal(robot, {:x, gx}) || closer_to_goal(robot, {:y, gy})) && obs == false ->
        robot = move(robot)
        Robotb.Actions.main("move")
        face_list = [:straight]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      Enum.member?(face_list, :right) ->
        if obs do
          robot = right(robot)
          Robotb.Actions.main("right")
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = move(robot)
          Robotb.Actions.main("move")
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        end

      Enum.member?(face_list, :left) ->
        if obs do
          robot = left(robot)
          Robotb.Actions.main("left")
          face_list = face_list ++ [:left]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = move(robot)
          Robotb.Actions.main("move")
          face_list = [:straight]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        end
      closer_to_goal(right(robot), {:x, gx}) || closer_to_goal(right(robot), {:y, gy}) ->
        robot = right(robot)
        Robotb.Actions.main("right")
        face_list = face_list ++ [:right]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      closer_to_goal(left(robot), {:x, gx}) || closer_to_goal(left(robot), {:y, gy}) ->
        robot = left(robot)
        Robotb.Actions.main("left")
        face_list = face_list ++ [:left]
        reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
      true ->
        if move(left(robot)) == left(robot) do
          robot = right(robot)
          Robotb.Actions.main("right")
          face_list = face_list ++ [:right]
          reach_goal(robot, gx, gy, cli_proc_name, ch2, face_list)
        else
          robot = left(robot)
          Robotb.Actions.main("left")
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

      iex> {:ok, robot} = Robotb.place(2, :b, :west)
      iex> Robotb.report(robot)
      {2, :b, :west}
  """
  def report(%Robotb.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%Robotb.Position{facing: facing} = robot) do
    %Robotb.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%Robotb.Position{facing: facing} = robot) do
    %Robotb.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%Robotb.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %Robotb.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
    }
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%Robotb.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %Robotb.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%Robotb.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %Robotb.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%Robotb.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %Robotb.Position{robot | x: x - 1}
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
