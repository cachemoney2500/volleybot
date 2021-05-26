using Plots
import RecipesBase: plot

function plot(dataset, ::Val{:q})
    c = dataset.controller
    plot(c.time, c.q[1])
    plot!(c.time, c.q[2])
    plot!(c.time, c.q[3])
end

function plot(dataset, ::Val{:dq})
    c = dataset.controller
    plot(c.time, c.dq[1])
    plot!(c.time, c.dq[2])
    plot!(c.time, c.dq[3])
end

function plot(dataset, ::Val{:trq})
    c = dataset.controller
    plot(c.time, c.cmdtrq[1])
    plot!(c.time, c.cmdtrq[2])
    plot!(c.time, c.cmdtrq[3])
    #plot!(size=(800, 800))
end

function plot(dataset, ::Val{:all})
    plot(
         plot(dataset, Val(:q)),
         plot(dataset, Val(:dq)),
         plot(dataset, Val(:trq)),
         layout=(3,1), size=(600, 1200)
    )
end
