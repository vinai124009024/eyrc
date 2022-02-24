defmodule PhserverWeb.PageController do
  use PhserverWeb, :controller

  def index(conn, _params) do
    render(conn, "index.html")
  end
end
