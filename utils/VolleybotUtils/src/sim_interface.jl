using Redis
using DynamicsAndControl
using Infiltrator

get_connection() = RedisConnection()

KEYS = Dict([
             # flags and sim
             :cflag => "cs225a::simulation::controller_start_flag",
             :k_control => "cs225a::controller::k_iter",
             :k_sim => "cs225a::simulation::k_iter",

             # ball and  configs
             :ballq => "cs225a::volleybot::ball::sensors::q",
             :balldq => "cs225a::volleybot::ball::sensors::dq",
             :toss_pos => "cs225a::volleybot::ball::toss_pos",
             :toss_vel => "cs225a::volleybot::ball::toss_vel",
             :hitpos_custom => "cs225a::volleybot::robot1::input::hit_pos",

             # robot state and control
             :cmdtrq => (i) -> "cs225a::volleybot::robot$(i)::actuators::fgc",
             :q => (i) -> "cs225a::volleybot::robot$(i)::sensors::q",
             :dq => (i) -> "cs225a::volleybot::robot$(i)::sensors::dq",
             :q_custom => (i) -> "cs225a::volleybot::robot$(i)::input::q_custom",
            ])

function setup()
    conn = RedisConnection()
    start = ()->set(conn, KEYS[:cflag], "true")
    stop = ()->set(conn, KEYS[:cflag], "false")
    return conn, start, stop
end

function record(conn)
    logger = LogDataSink()

    println("waiting for start...")
    while parse(Bool, get(conn, KEYS[:cflag])) == false
    end

    println("started. logging...")
    while parse(Bool, get(conn, KEYS[:cflag])) == true
        k_control = parse(Int, get(conn, KEYS[:k_control]))
        t_control = k_control * 0.001

        for robot=1:2
            cmdtrq = parse_eigen_array(get(conn, KEYS[:cmdtrq](robot)))
            q = parse_eigen_array(get(conn, KEYS[:q](robot)))
            dq = parse_eigen_array(get(conn, KEYS[:dq](robot)))
            ballq = parse_eigen_array(get(conn, KEYS[:ballq]))
            balldq = parse_eigen_array(get(conn, KEYS[:balldq]))
            output_name = Symbol(:controller, robot)
            log!(logger, output_name, t_control, (;cmdtrq, q, dq, ballq, balldq))
        end

        if k_control % 200 == 0
            println("\tt_control=$(t_control)")
        end

        #k_sim = parse(Int, get(conn, KEYS[:k_sim]))
        #t_sim = k_sim * 0.001
        #q = parse_eigen_array(get(conn, KEYS[:q]))
        #dq = parse_eigen_array(get(conn, KEYS[:dq]))
        #t_diff = t_sim - t_control
        #log!(logger,:sim, t_sim, (;q, dq, t_diff))
    end

    return DynamicsAndControl.SimulationDataset(logger)
end

function parse_eigen_array(arr_str)
    arr_str_stripped = arr_str[2:end-1]
    splitrow(row) = filter(el->length(el) > 0, split(row, [' ', ',']))
    rows = [parse.(Float64, splitrow(row))' for row in split(arr_str_stripped, ";")]

    if length(rows) > 1
        return vcat(rows...)
    else
        return rows[1]'
    end
end

function setarr(sym, conn, arr)
    set(conn, KEYS[sym], string(arr))
end

function getarr(sym, conn)
    parse_eigen_array(get(conn, KEYS[sym]))
end
