"""Plot saved robot and pedestrian tracks with sampled clearance histories."""
from pathlib import Path
import sys,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]
sys.path.insert(0,str(ROOT/'py'))
from sfm import Scenario
plt.rcParams.update({'font.size':11,'axes.titlesize':12,'axes.labelsize':11,'legend.fontsize':10,'font.family':'DejaVu Sans'})
def main():
 out=BASE/'figures';out.mkdir(exist_ok=True)
 manifest=json.loads((BASE/'trace_manifest.json').read_text())
 for name in manifest['inputs']:
  s=Scenario.load(ROOT/name)
  fig,axs=plt.subplots(3,2,figsize=(9,9.2),gridspec_kw={'height_ratios':[2.1,1,1.3]},layout='constrained')
  fig.get_layout_engine().set(rect=(0,.045,1,.91),hspace=.08)
  for col,arm in enumerate(['hybrid','hybrid-ttc']):
   base=BASE/'traces'/(Path(name).stem+'__'+arm);d=np.load(base.with_suffix('.npz'));r=json.loads(base.with_suffix('.json').read_text());x=d['positions']
   ax=axs[0,col]
   for o in s.obstacles:ax.add_patch(Polygon(o,fc='#dddddd',ec='#555555',lw=.5))
   for i in range(1,x.shape[2]):ax.plot(x[:,0,i],x[:,1,i],color='#888888',alpha=.35,lw=.4,label='Pedestrians' if i==1 else None)
   ax.plot(s.waypoints[0],s.waypoints[1],':',color='#cf7e18',lw=1,marker='.',ms=3,label='Waypoints')
   ax.plot(x[:,0,0],x[:,1,0],color='#145887',lw=1.3,label='Robot')
   ax.plot(*s.robot_start,'o',color='#145887',ms=4);ax.plot(*s.params.goal,'*',color='#217d44',ms=8)
   ax.set(xlim=(0,52),ylim=(0,52),xlabel='x (m)\n('+chr(97+col)+')',ylabel='y (m)',aspect='equal',title=('Guided SFM' if col==0 else 'Guided SFM + TTC')+'\n'+('Unsafe arrival' if r['outcome']=='success' and r['contacts'] else ('Safe arrival' if r['outcome']=='success' else r['outcome'].capitalize()))+', contact episodes '+str(r['contacts']))
   if col==0:fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.995),ncol=3,frameon=True,edgecolor='#555555',facecolor='white',framealpha=1)
   ax.tick_params(axis='both',labelsize=8)
   ax.xaxis.label.set_size(9);ax.yaxis.label.set_size(9)
   ax.title.set_fontsize(10)
   # Separate detail panel at closest logged pedestrian approach (positions sampled every 0.1 s).
   distances=np.linalg.norm(x[:,:,1:]-x[:,:,:1],axis=1);it,ip=np.unravel_index(np.argmin(distances),distances.shape);ip+=1
   center=x[it,:,0];ins=axs[1,col];ins.set_facecolor('white')
   for o in s.obstacles:ins.add_patch(Polygon(o,fc='#ddd',ec='#555',lw=.5))
   ins.plot(x[:,0,0],x[:,1,0],color='#145887',lw=.8)
   for i in range(1,x.shape[2]):ins.add_patch(Circle(x[it,:,i],s.params.rPed,fc='#888888',ec='#555555',alpha=.6,lw=.4))
   ins.add_patch(Circle(center,s.params.rRobot,fc='#145887',alpha=.8))
   ins.add_patch(Circle(center,s.params.rRobot+.15,fill=False,ec='#c44d24',ls='--',lw=1))
   ins.set(xlim=(center[0]-2,center[0]+2),ylim=(center[1]-2,center[1]+2),aspect='equal',xticks=[],yticks=[])
   ins.set_title('Closest logged approach: t = '+f'{d["position_times"][it]:.1f}'+' s',fontsize=10)
   ins.set_xlabel('('+chr(99+col)+')')
   ax=axs[2,col];t=d['gap_times'];ax.plot(t,d['ped_gap'],color='#145887',lw=.65,label='Pedestrian');ax.plot(t,d['obs_gap'],color='#b67513',lw=.65,ls='-.',label='Obstacle')
   ax.axhline(0,color='#9b2929',lw=.8);ax.axhline(.15,color='#666',ls='--',lw=.8,label='0.15 m');ax.fill_between(t,0,np.minimum(d['ped_gap'],d['obs_gap']),where=np.minimum(d['ped_gap'],d['obs_gap'])<0,color='#9b2929',alpha=.2)
   ax.set_yscale('symlog',linthresh=.2)
   ax.set(xlabel='Time (s)\n('+chr(101+col)+')',ylabel='Surface gap (m)',xlim=(0,max(t)))
   ax.set_yticks([-.2,0,.15,1,5,20]);ax.set_yticklabels(['-0.2','0','0.15','1','5','20'])
   if col==0:fig.legend(*ax.get_legend_handles_labels(),loc='lower center',bbox_to_anchor=(.5,0),ncol=3,frameon=True,edgecolor='#555555',facecolor='white',framealpha=1)
   ax.tick_params(axis='both',labelsize=8)
   ax.xaxis.label.set_size(9);ax.yaxis.label.set_size(9)
   ax.grid(alpha=.15)
  for panel in axs.flat:
   for spine in panel.spines.values():spine.set_linewidth(.45)
   panel.title.set_fontfamily('DejaVu Sans')
  fig.savefig(out/(s.condition+'.png'),dpi=220);plt.close(fig)
if __name__=='__main__':main()
