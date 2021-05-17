using LightXML
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
