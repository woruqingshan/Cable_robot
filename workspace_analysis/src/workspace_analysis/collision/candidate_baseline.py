"""Explicit research SRDF artifact builder from a frozen MoveIt candidate matrix."""
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from .errors import CollisionError
from .semantic import SemanticConfig,candidate_pairs


def build_candidate_baseline(model,strict_srdf,source_path,output_dir):
    """Create a NEW opt-in configuration; never edit strict/default configuration.

    Exactly Adjacent/Default are selected. Never/Always and remaining pairs stay
    checked. A research configuration is not a reviewed assembly permission.
    """
    strict=SemanticConfig.load(strict_srdf,model)
    source_bytes=Path(source_path).read_bytes();source_text=source_bytes.decode('utf-8')
    selected,geometry=candidate_pairs(source_text,model)
    root=ET.fromstring(strict.wire()['semantic']['srdf_xml'])
    for element in list(root):
        if element.tag=='disable_collisions':root.remove(element)
    root.insert(0,ET.Comment('EXPERIMENTAL: MoveIt candidates, no physical safety approval'))
    for (a,b),kind in sorted(selected.items()):ET.SubElement(root,'disable_collisions',link1=a,link2=b,reason='Experimental_MoveIt_'+kind+'_UNVERIFIED')
    ET.indent(root);xml=ET.tostring(root,encoding='unicode')+'\n'
    review={'id':'standard_moveit_candidate_v0','status':'candidate_baseline_unverified','usage':'research_only',
        'urdf_sha256':model.metadata()['expanded_urdf_sha256'],'source_geometry_sha256':geometry,'srdf_sha256':sha256(xml.encode()).hexdigest(),
        'candidate_source_sha256':sha256(source_bytes).hexdigest(),'candidate_source_origin':str(Path(source_path).resolve()),
        'selected_categories':['Adjacent','Default'],'disabled_pairs':[list(p) for p in sorted(selected)],
        'pair_reviews':[{'links':list(p),'candidate_class':kind,'decision':'candidate_disabled_unverified','evidence':'candidate_source.json; standard MoveIt classification only; physical review PENDING'} for p,kind in sorted(selected.items())],
        'normal_reference_status':'PENDING','hardware_validated':False,
        'strict_reference_srdf_sha256':strict.wire()['semantic']['srdf_sha256'],
        'excluded_categories':['Never','Always','Not Disabled']}
    config=SemanticConfig.from_xml(xml,model,review,allow_candidate=True,candidate_source_text=source_text)
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=False)
    for name,data in [('standard_moveit_candidate_v0.srdf',xml.encode()),('semantic_manifest.json',(json.dumps(review,indent=2)+'\n').encode()),('candidate_source.json',source_bytes)]:
        with (out/name).open('xb') as f:f.write(data)
    return config
