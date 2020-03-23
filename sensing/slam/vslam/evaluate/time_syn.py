import sys
with open('orbslam2_'+sys.argv[1]+'.tum','r') as f:
    orb = f.readlines()
with open('vins_fusion_'+sys.argv[1]+'.tum','r') as f:
    vins = f.readlines()

time_syn = float(vins[0].split(' ')[0])
time_init = float(orb[0].split(' ')[0])

orb_sym = open('orbslam2_syn_'+sys.argv[1]+'.tum', 'w')

for line in orb:
    line = line.split(' ')
    line[0] = str(float(line[0])-time_init+time_syn)
    orb_sym.write(' '.join(line))