using LinearAlgebra
using Printf

function predict(v_incident, v_swing, θ, dz)
    z_des = [cos(θ), 0.0, sin(θ)]
    v_incident_swing_frame = v_incident - v_swing
    vz = dot(-v_incident_swing_frame, normalize(z_des))*normalize(z_des)
    vperp = (-v_incident_swing_frame) - vz
    v_out_swing_frame = vz - vperp
    v_out = v_out_swing_frame + v_swing

    Δt = quadratic_eqn(-1/2*9.81, v_out[3], dz)

    dx = v_out[1]*Δt
    dy = v_out[2]*Δt

    return [dx, dy]
end

quadratic_eqn(a, b, c) = (-b - √(b^2 - 4*a*c))/2a

function iterative_solver(v_incident, v_swing, θ_init, dz, K, dxy_des)
    θ = θ_init
    for i=1:7
        dxy = predict(v_incident, v_swing, θ, dz)
        @printf "θ = %2.2f\t dxy = %2.2f\tdxy_des =%2.2f\n" θ dxy[1] dxy_des[1]
        θ += K*(dxy[1] - dxy_des[1])
    end
end
