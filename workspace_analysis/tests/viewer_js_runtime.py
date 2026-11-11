"""Run production replay JS in the existing system JavaScriptCore engine.

Only DOM/Plotly/timers are test doubles; no browser/WebGL or network access.
Uses Ubuntu desktop's existing library via stdlib ctypes, with no installation.
"""
import ctypes as c
import ctypes.util
import json


class JavaScriptRuntime:
    def __init__(self):
        path=ctypes.util.find_library('javascriptcoregtk-4.0')
        if path is None:raise RuntimeError('Viewer JS tests require the existing system libjavascriptcoregtk-4.0 runtime')
        self.lib=c.CDLL(path)
        signatures=[('JSGlobalContextCreate',[c.c_void_p],c.c_void_p),
            ('JSStringCreateWithUTF8CString',[c.c_char_p],c.c_void_p),
            ('JSEvaluateScript',[c.c_void_p,c.c_void_p,c.c_void_p,c.c_void_p,c.c_int,c.POINTER(c.c_void_p)],c.c_void_p),
            ('JSValueToStringCopy',[c.c_void_p,c.c_void_p,c.POINTER(c.c_void_p)],c.c_void_p),
            ('JSStringGetMaximumUTF8CStringSize',[c.c_void_p],c.c_size_t),
            ('JSStringGetUTF8CString',[c.c_void_p,c.c_char_p,c.c_size_t],c.c_size_t),
            ('JSStringRelease',[c.c_void_p],None),('JSGlobalContextRelease',[c.c_void_p],None)]
        for name,args,result in signatures:
            fn=getattr(self.lib,name);fn.argtypes=args;fn.restype=result
        self.context=self.lib.JSGlobalContextCreate(None)
        if not self.context:raise RuntimeError('JavaScriptCore context creation failed')

    def evaluate(self,script):
        string=self.lib.JSStringCreateWithUTF8CString(script.encode());error=c.c_void_p()
        try:value=self.lib.JSEvaluateScript(self.context,string,None,None,1,c.byref(error))
        finally:self.lib.JSStringRelease(string)
        result=self.lib.JSValueToStringCopy(self.context,error.value or value,None)
        try:
            length=self.lib.JSStringGetMaximumUTF8CStringSize(result);buffer=c.create_string_buffer(length)
            self.lib.JSStringGetUTF8CString(result,buffer,length);text=buffer.value.decode()
        finally:self.lib.JSStringRelease(result)
        if error.value:raise AssertionError('JavaScript evaluation failed: '+text)
        return text

    def json(self,expression):return json.loads(self.evaluate('JSON.stringify('+expression+')'))
    def close(self):
        if self.context:self.lib.JSGlobalContextRelease(self.context);self.context=None
    def __enter__(self):return self
    def __exit__(self,*args):self.close()


HARNESS=r'''
class Element {
 constructor(tag){this.tag=tag;this.children=[];this.textContent='';this.dataset=new Proxy({},{set(o,k,v){o[k]=String(v);return true;}});this._value='';}
 set value(v){this._value=String(v);} get value(){return this._value;}
 append(...items){this.children.push(...items);} setAttribute(k,v){this[k]=String(v);}
}
const created=[];
const plotRoot=new Element('plot');plotRoot.data=INPUT_TRACES;plotRoot.layout=INPUT_LAYOUT;plotRoot.parentNode={insertBefore(){}};
const document={getElementById(){return plotRoot;},createElement(tag){const e=new Element(tag);created.push(e);return e;}};
const window={};const intervals=new Map();let nextTimer=1;const renderHistory=[];const pendingReact=[];
function setInterval(callback){const id=nextTimer++;intervals.set(id,callback);return id;}
function clearInterval(id){intervals.delete(id);}
const Plotly={hold:false,failNext:false,react(r,data,layout){
 if(this.failNext){this.failNext=false;return Promise.reject(new Error('test Plotly failure'));}
 const commit=()=>{r.data=data;r.layout=layout;renderHistory.push(layout.meta.path_sample_index);};
 if(this.hold)return new Promise(resolve=>pendingReact.push(()=>{commit();resolve();}));
 commit();return Promise.resolve();
}};
function getSlider(){return created.find(e=>e.tag==='input'&&e.type==='range');}
function getPlay(){return created.find(e=>e.tag==='button');}
function getPanel(){return JSON.parse(created.find(e=>e.tag==='pre').textContent);}
function tick(){for(const callback of [...intervals.values()])callback();}
function flushReact(){const done=pendingReact.shift();if(done)done();}
function selectFrame(index){getSlider().value=index;getSlider().oninput();}
function checkFrame(index){
 const s=window.pathViewer.model.states[index],panel=getPanel();
 if(+root.dataset.pathIndex!==index||+getSlider().value!==index||panel.sample_index!==index)throw Error('frame controls/panel out of sync');
 if(JSON.stringify(panel.q)!==JSON.stringify(s.q)||JSON.stringify(root.layout.meta.q)!==JSON.stringify(s.q))throw Error('q out of sync');
 if(root.dataset.pathValid!==String(s.valid)||root.dataset.replayError)throw Error(root.dataset.replayError||'valid flag out of sync');
 const tcp=root.data.find(t=>t.name==='Actual synthetic TCP');
 const m=window.pathViewer.model,T=s.T_base_to_ee,tool=m.T_ee_tcp;
 const p=T.slice(0,3).map(row=>row[0]*tool[0][3]+row[1]*tool[1][3]+row[2]*tool[2][3]+row[3]);
 if(tcp&&Math.max(...[tcp.x[0]-p[0],tcp.y[0]-p[1],tcp.z[0]-p[2]].map(Math.abs))>1e-11)throw Error('TCP out of sync');
 const meshes=root.data.filter(t=>(t.meta||{}).role==='robot_collision_mesh');
 m.surfaces.forEach((surface,i)=>{const t=s.link_transforms_base[surface.link],v=surface.vertices[0];const p=t.slice(0,3).map(row=>row[0]*v[0]+row[1]*v[1]+row[2]*v[2]+row[3]);
  if(Math.max(...[meshes[i].x[0]-p[0],meshes[i].y[0]-p[1],meshes[i].z[0]-p[2]].map(Math.abs))>1e-11)throw Error('mesh out of sync');});
 const expected=root.data.find(t=>t.name==='Approach expected TCP');
 if(expected&&s.expected_T_base_tcp){const p=s.expected_T_base_tcp.slice(0,3).map(row=>row[3]);if(Math.max(...[expected.x[0]-p[0],expected.y[0]-p[1],expected.z[0]-p[2]].map(Math.abs))>1e-11)throw Error('expected TCP out of sync');}
 const trajectory=root.data.find(t=>(t.meta||{}).role==='checked_tcp_path');
 if(trajectory&&trajectory.x.length!==m.states.length)throw Error('trajectory was truncated');
 return {index,meshes:meshes.length,panel_q:panel.q,only_endpoint_is_task_constrained:panel.only_endpoint_is_task_constrained};
}
'''
