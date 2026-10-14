import math, subprocess, time
R='tron1a'; Z=0.82; STEP=0.08
WP=[(1.26,-1.00),(1.26,-2.40)]
def pose():
    o=subprocess.run(['gz','model','-m',R,'-p'],capture_output=True,text=True,timeout=5).stdout.split()
    return float(o[0]),float(o[1])
def put(x,y,yaw):
    subprocess.run(['gz','model','-m',R,'-x',f'{x:.3f}','-y',f'{y:.3f}','-z',str(Z),'-R','0','-P','0','-Y',f'{yaw:.3f}'],timeout=5)
x,y=pose()
for gx,gy in WP:
    yaw=math.atan2(gy-y,gx-x)
    put(x,y,yaw); time.sleep(0.5)
    while math.hypot(gx-x,gy-y)>0.05:
        d=math.hypot(gx-x,gy-y); s=min(STEP,d)
        x+=s*(gx-x)/d; y+=s*(gy-y)/d
        put(x,y,yaw); time.sleep(0.03)
    print('도착',gx,gy)
print('최종',pose())
