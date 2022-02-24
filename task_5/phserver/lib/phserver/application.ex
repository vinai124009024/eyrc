defmodule Phserver.Application do
  # See https://hexdocs.pm/elixir/Application.html
  # for more information on OTP Applications
  @moduledoc false

  use Application

  @impl true
  def start(_type, _args) do
    children = [
      # Start the Telemetry supervisor
      PhserverWeb.Telemetry,
      # Start the PubSub system
      {Phoenix.PubSub, name: Phserver.PubSub},
      # Start the Endpoint (http/https)
      PhserverWeb.Endpoint,
      # Start a worker by calling: Phserver.Worker.start_link(arg)
      # {Phserver.Worker, arg}
      {Phserver.Timer, []}
    ]

    # See https://hexdocs.pm/elixir/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: Phserver.Supervisor]
    Supervisor.start_link(children, opts)
  end

  # Tell Phoenix to update the endpoint configuration
  # whenever the application is updated.
  @impl true
  def config_change(changed, _new, removed) do
    PhserverWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
