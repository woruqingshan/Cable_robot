"""Declared gripper URDF tree and explicit, unverified physical interpretations."""
from .geometry_audit import topology


def assembly_topology(model,root='gripper_link'):
    links={root};pending=[root]
    while pending:
        parent=pending.pop()
        for joint in model.joints.values():
            if joint.parent==parent:links.add(joint.child);pending.append(joint.child)
    edges=[{'joint':j.name,'parent':j.parent,'child':j.child,'type':j.kind,'origin_xyz':j.xyz,'origin_rpy':j.rpy,'axis':j.axis,'limits':[j.lower,j.upper],'mimic':{'joint':j.mimic_joint,'multiplier':j.multiplier,'offset':j.offset} if j.mimic_joint else None} for j in model.joints.values() if j.child in links or j.child==root]
    return {'root':root,'members':sorted(links),'urdf_tree':edges,'physical_mechanism':'Gear engagement and linkage closure are candidates from source geometry, not extra URDF loop constraints. Mimic is not physical validation.','physical_closed_loop_verified':False}
