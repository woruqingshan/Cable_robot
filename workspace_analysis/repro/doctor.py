"""Dependency and portable model identity diagnostics. No installation or controls."""
from hashlib import sha256
import importlib
import importlib.metadata
import ctypes.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

MODULES={'numpy':'numpy','scipy':'scipy','yaml':'PyYAML','plotly':'plotly','PyKDL':'PyKDL',
         'urdf_parser_py':'urdf-parser-py','pytest':'pytest'}
ROS_MODULES={'xacro':'xacro','ament_index_python':'ament-index-python','rclpy':'rclpy','tf2_ros':'tf2-ros'}
ROS_PACKAGES=['robot_state_publisher','tf2_ros','tf2_ros_py','moveit_core','moveit_ros_planning',
              'moveit_setup_framework','moveit_setup_srdf_plugins','resource_retriever','urdf','srdfdom']
APT_PACKAGES=['python3-numpy','python3-scipy','python3-pykdl','python3-yaml','python3-pytest',
 'libjavascriptcoregtk-4.0-18','ros-humble-ros-base','ros-humble-xacro','ros-humble-urdf-parser-py',
 'ros-humble-ament-index-python','ros-humble-moveit-core','ros-humble-moveit-ros-planning',
 'ros-humble-moveit-setup-framework','ros-humble-moveit-setup-srdf-plugins','libfcl-dev',
 'libfcl0.7','liborocos-kdl-dev','liborocos-kdl1.5','libeigen3-dev','nlohmann-json3-dev','libssl-dev','libtinyxml2-dev',
 'ros-humble-robot-state-publisher','ros-humble-tf2-ros','ros-humble-tf2-ros-py','ros-humble-rclpy',
 'ros-humble-resource-retriever','ros-humble-urdf','ros-humble-srdfdom','gcc','g++','cmake']


def digest(path):return sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def check(name,status,summary,**details):return {'id':name,'status':status,'summary':summary,'details':details}


def probe_modules(modules):
    rows=[]
    for name,distribution in modules.items():
        try:
            module=importlib.import_module(name)
            try:version=importlib.metadata.version(distribution)
            except importlib.metadata.PackageNotFoundError:version=getattr(module,'__version__',None)
            rows.append(check('python.'+name,'PASS','importable',version=version,version_status='RESOLVED' if version else 'UNRESOLVED',
                              origin=getattr(module,'__file__',None),distribution=distribution))
        except (ImportError,OSError) as exc:rows.append(check('python.'+name,'BLOCKED','dependency unavailable',error=str(exc)))
        except Exception as exc:rows.append(check('python.'+name,'FAIL','installed dependency import is incompatible',error=str(exc)))
    return rows


def probe_sources(root):
    # Lazy imports keep Core free of ROS/Native initialization.
    modules=['geometry.se3','scene.schema','task.targets','planning.joint_path','planning.rrt_connect','viz.joint_path']
    rows=[]
    for name in modules:
        try:
            module=importlib.import_module('workspace_analysis.'+name);path=Path(module.__file__).resolve()
            if not path.is_relative_to(root/'src'):raise ValueError('imported another checkout: '+str(path))
            rows.append(check('source.'+name,'PASS','local source imports',origin=str(path)))
        except (ImportError,OSError) as exc:rows.append(check('source.'+name,'BLOCKED','source dependency unavailable',error=str(exc)))
        except Exception as exc:rows.append(check('source.'+name,'FAIL','source import failed',error=str(exc)))
    return rows


def portable_robot_identity(model):
    """Hash loaded XML semantics with resource filenames replaced by content IDs.

    Original raw URDF/source paths are retained separately. ALL visual/collision
    mesh refs, collision origin/scale, full tree/limits/mimic and source includes
    remain in the identity. No official model or binding is rewritten.
    """
    from workspace_analysis.collision.protocol import mesh_resource_path
    metadata=model.metadata();root=ET.fromstring(model.urdf_xml);meshes=[]
    for link in root.findall('link'):
        for kind in ['visual','collision']:
            for index,item in enumerate(link.findall(kind)):
                mesh=item.find('geometry/mesh')
                if mesh is None:continue
                uri=mesh.get('filename');path=mesh_resource_path(uri).resolve()
                if not path.is_file():raise FileNotFoundError(str(path))
                h=digest(path);meshes.append({'link':link.get('name'),'kind':kind,'index':index,'sha256':h,'path':str(path),'uri':uri})
                mesh.set('filename','sha256:'+h)
    def node(element):
        return {'tag':element.tag,'attributes':dict(sorted(element.attrib.items())),'text':(element.text or '').strip(),
                'children':[node(child) for child in element]}
    source=Path(metadata['source_path']).resolve();package_root=source.parent.parent
    includes={}
    for path,h in metadata['source_hashes'].items():
        p=Path(path).resolve()
        if not p.is_relative_to(package_root):raise ValueError('include outside package root requires explicit provenance: '+str(p))
        includes[p.relative_to(package_root).as_posix()]=h
    resources=[{k:m[k] for k in ['link','kind','index','sha256']} for m in meshes]
    return {'raw_urdf_sha256':metadata['expanded_urdf_sha256'],'portable_model_sha256':sha256(canonical(node(root)).encode()).hexdigest(),
        'source_path':str(source),'package_root':str(package_root),'include_sha256':includes,
        'base_frame':model.base_frame,'ee_frame':model.ee_frame,'joint_order':list(model.joint_order),
        'chain':metadata['chain'],'auxiliary_positions':metadata['auxiliary_positions'],'xacro_environment':metadata['xacro_environment'],'resources':resources,'resource_locations':meshes,
        'collision_mesh_count':sum(m['kind']=='collision' for m in meshes)}


def compare_robot_identity(actual,expected):
    keys=['portable_model_sha256','include_sha256','resources','base_frame','ee_frame','joint_order','chain','auxiliary_positions','xacro_environment','collision_mesh_count','model_config_sha256']
    different=[key for key in keys if canonical(actual.get(key))!=canonical(expected.get(key))]
    return check('robot.identity','FAIL' if different else 'PASS','model/resource content differs' if different else 'content identity verified',
        differences=different,raw_urdf_matches=actual['raw_urdf_sha256']==expected.get('raw_urdf_sha256'),
        portable_geometry_matches=actual['portable_model_sha256']==expected.get('portable_model_sha256'),
        include_content_matches=actual['include_sha256']==expected.get('include_sha256'),
        resource_content_matches=actual['resources']==expected.get('resources'),
        source_path_changed=actual['source_path']!=expected.get('source_path'),identity=actual,
        note='raw path/serialization changes alone do not fail geometry identity; SRDF binding is independently checked')


def load_robot(root,model_path=None):
    from workspace_analysis.robot.model import load_model
    if model_path is not None:
        if not Path(model_path).is_file():raise FileNotFoundError(str(model_path))
    else:
        from ament_index_python.packages import get_package_share_directory
        try:share=Path(get_package_share_directory('jetrover_description'))
        except LookupError as exc:raise FileNotFoundError('ament jetrover_description: '+str(exc)) from exc
        if not (share/'urdf/jetrover.xacro').is_file():raise FileNotFoundError(str(share))
    return load_model(root/'config/workspace.yaml',model_path=model_path)


def toolchain_snapshot():
    commands={'compiler':['c++','--version'],'cmake':['cmake','--version'],'apt':['dpkg-query','-W','-f','${binary:Package}\t${Version}\t${db:Status-Status}\n',*APT_PACKAGES]}
    result={}
    for name,args in commands.items():
        try:
            p=subprocess.run(args,capture_output=True,text=True,timeout=15)
            result[name]={'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr,'version_status':'OBSERVED' if p.stdout else 'UNRESOLVED'}
        except (OSError,subprocess.TimeoutExpired) as exc:result[name]={'error':str(exc),'version_status':'UNRESOLVED'}
    return result


def probe_ros_packages():
    rows=[]
    for name in ROS_PACKAGES:
        try:
            from ament_index_python.packages import get_package_share_directory
            share=Path(get_package_share_directory(name));manifest=share/'package.xml'
            version=ET.parse(manifest).findtext('version')
            rows.append(check('ros.package.'+name,'PASS','ament package available',path=str(share),version=version,manifest_sha256=digest(manifest)))
        except Exception as exc:rows.append(check('ros.package.'+name,'BLOCKED','required ROS package unavailable',error=str(exc)))
    return rows


def native_probe(model,scene,semantic,backend):
    from workspace_analysis.collision.client import CollisionClient
    with CollisionClient(backend,timeout=30) as client:
        ready=client.initialize(model,scene,semantic=semantic)
        identity=client.identity
    if not client.cleanup['exit_normal']:raise RuntimeError('Native initialization cleanup abnormal: '+str(client.cleanup))
    return {'identity':identity,'cleanup':client.cleanup,'detector':ready['audit']['detector'],
            'loaded_shapes':ready['audit']['moveit_shape_count'],'queries':0}


def run_doctor(root,mode='core',*,backend=None,model_path=None,assets_file=None):
    root=Path(root).resolve()
    if mode not in ['core','full']:raise ValueError('mode must be core or full')
    rows=[check('os','PASS','operating system observed',os_release=platform.freedesktop_os_release(),architecture=platform.machine()),
        check('python','PASS' if sys.version_info[:2]==(3,10) else 'FAIL','supported baseline: Python 3.10',
              version=platform.python_version(),executable=sys.executable,prefix=sys.prefix,base_prefix=sys.base_prefix)]
    rows+=probe_modules(MODULES);rows+=probe_sources(root)
    js=ctypes.util.find_library('javascriptcoregtk-4.0')
    rows.append(check('javascript.runtime','PASS' if js else 'BLOCKED','Viewer dynamic regression engine',library=js))
    full_ids=['ros','ros.packages','robot.identity','scene','srdf.strict','srdf.candidate','native','toolchain']
    if mode=='core':
        rows.extend(check(name,'NOT_CHECKED','Full mode only') for name in full_ids)
    else:
        rows+=probe_modules(ROS_MODULES)
        rows+=probe_ros_packages()
        rows.append(check('ros','PASS' if os.environ.get('ROS_DISTRO')=='humble' else 'BLOCKED','requires sourced ROS2 Humble',
            environment={k:os.environ.get(k) for k in ['ROS_DISTRO','ROS_VERSION','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH']}))
        tools=toolchain_snapshot();rows.append(check('toolchain','PASS' if all(tools[n].get('returncode')==0 for n in ['compiler','cmake']) else 'BLOCKED','compiler/CMake/APT inventory',snapshot=tools))
        model=None;scene=None;candidate=None
        try:
            model=load_robot(root,model_path);actual=portable_robot_identity(model)
            actual['model_config_sha256']=digest(root/'config/workspace.yaml')
            expected=json.loads(Path(assets_file or root/'repro/environment/model_assets.json').read_text())
            rows.append(compare_robot_identity(actual,expected))
        except (FileNotFoundError,ImportError) as exc:rows.append(check('robot.identity','BLOCKED','model/assets unavailable',error=str(exc)))
        except Exception as exc:rows.append(check('robot.identity','FAIL','model load/identity failed',error=str(exc)))
        try:
            from workspace_analysis.scene.schema import load_scene
            scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
            rows.append(check('scene','PASS','S1 loads',canonical_sha256=scene.canonical_scene_sha256,raw_sha256=scene.raw_yaml_sha256,objects=len(scene.collision_objects),ports=len(scene.ports)))
        except Exception as exc:rows.append(check('scene','FAIL','S1 load failed',error=str(exc)))
        for label,path in [('strict','jetrover_semantic_v0.srdf'),('candidate','standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf')]:
            if model is None:rows.append(check('srdf.'+label,'BLOCKED','requires loaded model'));continue
            try:
                from workspace_analysis.collision.semantic import SemanticConfig
                p=root/'config/collision/semantic'/path;s=SemanticConfig.load(p,model,allow_candidate=label=='candidate')
                if label=='candidate':candidate=s
                rows.append(check('srdf.'+label,'PASS','binding and provenance verified',source_sha256=digest(p),identity=s.wire()['semantic'],usage='RESEARCH_ONLY'))
            except Exception as exc:rows.append(check('srdf.'+label,'FAIL','SRDF binding fails; no identity bypass',error=str(exc)))
        binary=None if backend is None else Path(backend).expanduser().resolve()
        if binary is None or not binary.is_file() or not os.access(binary,os.X_OK):rows.append(check('native','BLOCKED','set --backend or C1_BACKEND to executable',path=None if binary is None else str(binary)))
        elif model is None or scene is None or candidate is None:rows.append(check('native','BLOCKED','requires model/Scene/SRDF'))
        elif any(c['id']=='robot.identity' and c['status']!='PASS' for c in rows):rows.append(check('native','BLOCKED','model identity did not pass'))
        else:
            try:rows.append(check('native','PASS','real Native FCL initialization verified (no configuration queries)',path=str(binary),sha256=digest(binary),audit=native_probe(model,scene,candidate,binary)))
            except Exception as exc:rows.append(check('native','FAIL','Native initialization/identity failed',path=str(binary),sha256=digest(binary),error=str(exc)))
    failed=any(r['status']=='FAIL' for r in rows);blocked=any(r['status']=='BLOCKED' for r in rows)
    status='FAIL' if failed else 'BLOCKED' if blocked else 'READY_FOR_CORE_TESTS' if mode=='core' else 'READY_FOR_RESEARCH_REPRODUCTION'
    return {'schema_version':1,'mode':mode,'status':status,'exit_code':2 if failed else 3 if blocked else 0,'usage':'RESEARCH_ONLY',
            'hardware_validated':False,'historical_artifacts_checked':False,'checks':rows}
