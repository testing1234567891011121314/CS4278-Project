"""Classify sweep logs: python summarize_sweep.py LOGDIR (expects o{idx}_s{seed}.log)."""
import json,glob,re,collections,sys,os
os.chdir(sys.argv[1] if len(sys.argv)>1 else ".")
names="master_chef_can sugar_box tomato_soup_can mustard_bottle potted_meat_can banana strawberry apple bleach_cleanser bowl mug plate large_marker hammer foam_brick".split()
for s in (42,7):
  for i in range(15):
    f=f"o{i}_s{s}.log"; t=open(f,errors='replace').read()
    m=[l for l in t.splitlines() if l.startswith("RESULT:")]
    if not m:
        cls="TIMEOUT" if "exit=124" in t else "CRASH"
        print(i,names[i],s,cls,"\n".join(t.splitlines()[-16:-1]) if cls=="CRASH" else "");continue
    r=json.loads(m[-1][7:]); ps=r.get("planner_stats") or {}
    st=[x[1] for x in r["states"]]
    if r.get("stopped")=="held object 100 steps" or (3 in st and r.get("final_state")==3): cls="PICK_OK"
    elif ps.get("reason")=="invalid_start_state": cls="START_INVALID"
    elif 2 in st and 4 in st: cls="EXEC_FAIL"
    elif ps.get("reason"): cls="SEARCH_EXHAUSTED"
    else: cls="OTHER"
    k={x:ps.get(x) for x in("reason","tried","ik_fail","approach_fail","opening_fail","closure_fail","lift_fail","transit_fail","start_state_error")}
    print(i,names[i],s,cls,"final",r.get("final_state"),"states",st,"gerr",r.get("grasp_error"),"lift",r.get("lift_cm"),"stopped",r.get("stopped"),k)
