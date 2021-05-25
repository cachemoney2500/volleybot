using LightXML
using Printf
using Infiltrator

function create_attached_volleybot_urdf()
    orig_doc = parse_file("../../jumping_panda.urdf")
    orig_root = root(orig_doc)

    new_doc = XMLDocument()
    new_root = create_root(new_doc, "robot")
    set_attribute(new_root, "name", "mmp_panda")

    for link in orig_root["link"]
        if attribute(link, "name") in ["ground_link", "base_x", "base_y", "base_heading", "foot"]
            unlink(link)
            add_child(new_root, link)
        end
    end

    for joint in orig_root["joint"]
        if attribute(joint, "name") in ["base_prismatic_x", "base_prismatic_y", "base_revolute", "base_pitch"]
            unlink(joint)
            add_child(new_root, joint)
        end
    end

    save_file(new_doc, "test.urdf")
end

function convert_urdf_coordinates(link_name, lower_joint_name)
    orig_doc = parse_file("../../jumping_panda.urdf")
    r = root(orig_doc)

    link = get_element(r, "link", link_name)
    lower_joint = get_element(r, "joint", lower_joint_name)

    joint_coordinate_transform(link, lower_joint)
end

function get_element(root, tag, name)
    return first(filter(el->attribute(el, "name") == name, root[tag]))
end

function joint_coordinate_transform(link, lower_joint)
    str_input(input) = parse.(Float64, split(input))
    inertial_origin = str_input(attribute(link["inertial"][1]["origin"][1], "xyz"))
    joint_origin = str_input(attribute(lower_joint["origin"][1], "xyz"))

    new_inertial = joint_origin - inertial_origin
    @printf "new inertial: \"%2.7f %2.7f %2.7f\"\n" new_inertial[1] new_inertial[2] new_inertial[3]
    @printf "new joint: \"%2.7f %2.7f %2.7f\"\n" -joint_origin[1] -joint_origin[2] -joint_origin[3]

    if length(link["visual"][1]["origin"]) > 0
        visual_origin = str_input(attribute(link["visual"][1]["origin"][1], "xyz"))
        new_visual = joint_origin - visual_origin
        @printf "new visual: \"%2.7f %2.7f %2.7f\"\n" new_visual[1] new_visual[2] new_visual[3]
    end
end
