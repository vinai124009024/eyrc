defmodule RobotbTest do
  use ExUnit.Case
  doctest Robotb

  test "greets the world" do
    assert Robotb.hello() == :world
  end
end
