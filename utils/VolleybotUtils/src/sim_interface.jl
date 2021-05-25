using Redis
using DynamicsAndControl
using REPL
using Infiltrator

get_connection() = RedisConnection()

KEYS = Dict([
             :cflag => "cs225a::simulation::controller_start_flag",
             :t_control => "cs225a::controller::time",
             :cmdtrq => "cs225a::volleybot::robot1::actuators::fgc",
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

    while parse(Bool, get(conn, KEYS[:cflag])) == true
        t_control = parse(Float64, get(conn, KEYS[:t_control]))
        cmdtrq = parse_eigen_array(get(conn, KEYS[:cmdtrq]))
        log!(logger,:controller, t_control, (;cmdtrq))
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
