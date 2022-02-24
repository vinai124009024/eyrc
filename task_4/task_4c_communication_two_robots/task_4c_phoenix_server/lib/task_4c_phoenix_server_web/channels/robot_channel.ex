defmodule Task4CPhoenixServerWeb.RobotChannel do
  use Phoenix.Channel

  @doc """
  Handler function for any Client joining the channel with topic "robot:status".
  Subscribe to the topic named "robot:update" on the Phoenix Server using Endpoint.
  Reply or Acknowledge with socket PID received from the Client.
  """
  def join("robot:status", _params, socket) do
    Task4CPhoenixServerWeb.Endpoint.subscribe("robot:update")
    {:ok, socket}
  end

  @doc """
  Callback function for messages that are pushed to the channel with "robot:status" topic with an event named "new_msg".
  Receive the message from the Client, parse it to create another Map strictly of this format:
  %{"client" => < "robot_A" or "robot_B" >,  "left" => < left_value >, "bottom" => < bottom_value >, "face" => < face_value > }

  These values should be pixel locations for the robot's image to be displayed on the Dashboard
  corresponding to the various actions of the robot as recevied from the Client.

  Broadcast the created Map of pixel locations, so that the ArenaLive module can update
  the robot's image and location on the Dashboard as soon as it receives the new data.

  Based on the message from the Client, determine the obstacle's presence in front of the robot
  and return the boolean value in this format {:ok, < true OR false >}.

  If an obstacle is present ahead of the robot, then broadcast the pixel location of the obstacle to be displayed on the Dashboard.
  """

  def handle_in("new_msg", message, socket) do

    # determine the obstacle's presence in front of the robot and return the boolean value
    is_obs_ahead = Task4CPhoenixServerWeb.FindObstaclePresence.is_obstacle_ahead?(message["x"], message["y"], message["face"])

    # file object to write each action taken by each Robot (A as well as B)
    {:ok, out_file} = File.open("task_4c_output.txt", [:append])
    # write the robot actions to a text file
    IO.binwrite(out_file, "#{message["client"]} => #{message["x"]}, #{message["y"]}, #{message["face"]}\n")

    ###########################
    ## complete this funcion ##
    ###########################
    y_to_num = %{"a" => 1, "b" => 2, "c" => 3, "d" => 4, "e" => 5, "f" => 6}
    client_ = (message["client"])
    new_x = (message["x"]-1)*150
    new_y = (Map.get(y_to_num, message["y"])-1)*150
    new_face = "robot_facing_#{message["face"]}.png"
    obspos = if is_obs_ahead do
      cond do
        message["face"] == "north" -> {new_x, new_y + 75}
        message["face"] == "south" -> {new_x, new_y - 75}
        message["face"] == "east" -> {new_x + 75, new_y}
        message["face"] == "west" -> {new_x - 75, new_y} 
        true -> nil
      end
    else
      nil
    end
    
    if client_ == "robot_A" do
      Agent.update(:curr_pos, fn l -> %{"a" => message, "b" => l["b"]} end)
    else
      Agent.update(:curr_pos, fn l -> %{"a" => l["a"], "b" => message} end)
    end
    :ok = Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", %{client: client_, x: new_x, y: new_y, face: new_face, obs: obspos})
    {:reply, {:ok, is_obs_ahead}, socket}
  end

  #########################################
  ## define callback functions as needed ##
  #########################################

  def handle_in("init", message, socket) do
    Agent.start_link(fn -> [[], []] end, name: :hello)
    Agent.start_link(fn -> message end, name: :goals)
    Agent.start_link(fn -> %{"a" => %{}, "b" => %{}} end, name: :curr_pos)
    {:reply, :ok, socket}
  end

  def handle_in("get_goals", _message, socket) do
    data = File.read!("Plant_Positions.csv")
    data = String.split(data, "\n")
    [_|data] = data
    data_s = Enum.map(data, fn d -> %{ "num" => String.split(d, ",") |> Enum.at(0), "task" => "sowing", "pos" => String.split(d, ",") |> Enum.at(0) |> String.to_integer() |> goal_conv()} end)
    data_w = Enum.map(data, fn d -> %{ "num" => String.split(d, ",") |> Enum.at(1), "task" => "weeding", "pos" => String.split(d, ",") |> Enum.at(1) |> String.to_integer() |> goal_conv()} end)
    data = data_s ++ data_w
    Agent.start_link(fn -> data end, name: :goal_locs)
    {:reply, {:ok, data}, socket}
  end

  def goal_conv(g) do
    num_to_y = %{1 => :a, 2 => :b, 3 => :c, 4 => :d, 5 => :e, 6 => :f}
      [x,y] = 
      if rem(g, 5) == 0 do
        [5, g/5]
      else
        [rem(g, 5), (g / 5) + 1]
      end
      [Integer.to_string(x), Map.get(num_to_y, trunc(y))]
  end

  def startpos(as, bs) do
    Agent.update(:hello, fn l -> [as, bs] end)
  end

  def handle_in("get_start_pos", m, socket) do
    l = if m == "robotA" do
      Agent.get(:hello, fn l -> Enum.at(l, 0) end)
    else
      Agent.get(:hello, fn l -> Enum.at(l, 1) end)
    end
    {:reply, {:ok, l}, socket}
  end

  def handle_in("stop_process", _m, socket) do
    Task4CPhoenixServerWeb.Endpoint.broadcast("timer:stop", "stop_timer", %{})
    {:reply, :ok, socket}
  end

  def handle_in("get_upd_goals", _m, socket) do
    l = Agent.get(:goals, fn l -> l end)
    {:reply, {:ok, l}, socket}    
  end

  def handle_in("upd_goals", [g, r], socket) do
    g1 = Enum.at(g, 0) |> Integer.to_string()
    g2 = Enum.at(g, 1)
    :ok = Agent.update(:goals, fn l -> Enum.filter(l, fn g -> g != [g1, g2] end) end)
    loc = Enum.filter(Agent.get(:goal_locs, fn l -> l end), fn g -> g["pos"] == [g1, String.to_atom(g2)]  end) |> Enum.at(0)
    :ok = Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", {:goalpos, [r, loc]})
    {:reply, :ok, socket}    
  end

  def handle_in("get_pos", m, socket) do
    pos = if m == "robotB" do
      Agent.get(:curr_pos, fn l -> l["b"] end)
    else
      Agent.get(:curr_pos, fn l -> l["a"] end)
    end
    {:reply, {:ok, pos}, socket} 
  end
   
end
