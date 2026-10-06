import random, numpy as np
from scipy.spatial import cKDTree

random.seed(7)
V = []
for line in open('models/ground/mesh/terrain.obj'):
    if line.startswith('v '):
        p = line.split(); V.append([float(p[1]),float(p[2]),float(p[3])])
V = np.array(V); kt = cKDTree(V[:,:2])
cx, cy = V[:,0].mean(), V[:,1].mean()
R, NCL, CR = 32.0, 7, 9.0

ASSETS = [("Oak tree","Oak tree",30,0.7,1.2),
          ("Pine Tree","Pine Tree",30,0.7,1.2),
          ("falling_rock1","Falling Rock 1",90,0.8,2.5),
          ("small_rock_fall","Small Rock Fall",120,2.0,6.0)]

clumps = [(cx+random.uniform(-R,R), cy+random.uniform(-R,R)) for _ in range(NCL)]
gz = lambda x,y: float(V[kt.query([x,y],k=4)[1]][:,2].mean())

out, n = [], 0
for name, folder, count, smin, smax in ASSETS:
    for _ in range(count):
        ccx, ccy = random.choice(clumps)
        x = ccx + random.gauss(0, CR); y = ccy + random.gauss(0, CR)
        n += 1
        out.append(f"""    <include>
      <name>{name.replace(' ','_')}_{n}</name>
      <uri>model://{folder}</uri>
      <pose>{x:.2f} {y:.2f} {gz(x,y)-0.15:.2f} 0 0 {random.uniform(0,6.28):.3f}</pose>
    </include>""")

src = open('models/ground/test.world').read()
open('forest.world','w').write(src.replace('</world>', '\n'.join(out)+'\n    </world>'))
print(f"placed {n} objects + 14 grass patches")
