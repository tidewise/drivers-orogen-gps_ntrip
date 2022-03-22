# frozen_string_literal: true

using_task_library "gps_ntrip"

Robot.controller do
    Roby.plan.add_mission_task OroGen.gps_ntrip.Task.deployed_as_unmanaged("ntrip_client")
end
