using Plots
import RecipesBase: plot

function plot(c, ::Val{:q})
    plot(c.time, c.q[1])
    plot!(c.time, c.q[2])
    plot!(c.time, c.q[3])
end

function plot(c, ::Val{:dq})
    plot(c.time, c.dq[1])
    plot!(c.time, c.dq[2])
    plot!(c.time, c.dq[3])
end

function plot(c, ::Val{:trq})
    plot(c.time, c.cmdtrq[1])
    plot!(c.time, c.cmdtrq[2])
    plot!(c.time, c.cmdtrq[3])
    #plot!(size=(800, 800))
end

function plot(c, ::Val{:all})
    plot(
         plot(c, Val(:q)),
         plot(c, Val(:dq)),
         plot(c, Val(:trq)),
         layout=(3,1), size=(600, 1200)
    )
end

function plot(c, ::Val{:heading})
    plot(
        plot(c.time, c.q[4]*180/π),
        plot(c.time, c.dq[4]*180/π),
        plot(c.time, c.cmdtrq[4]),
        layout=(3,1), size=(600, 1200)
    )
end

function plot(c, ::Val{:joint}, i)
    plot(
        plot(c.time, c.q[i]*180/π),
        plot(c.time, c.dq[i]*180/π),
        plot(c.time, c.cmdtrq[i]),
        layout=(3,1), size=(600, 1200)
    )
end

function plot(c, ::Val{:ball})
    plot(
         begin
            plot(c.time, c.ballq[1])
            plot!(c.time, c.ballq[2])
            plot!(c.time, c.ballq[3])
        end,
        begin
            plot(c.time, c.balldq[1])
            plot!(c.time, c.balldq[2])
            plot!(c.time, c.balldq[3])
        end,
        layout=(2,1), size=(600, 800)
       )
end
