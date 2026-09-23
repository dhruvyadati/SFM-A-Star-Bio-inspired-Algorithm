"""Reformat two saved-result plots without changing their numeric computations."""
from pathlib import Path
import ast,sys
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
sys.path.insert(0,str(ROOT/'py'))
# Load the existing plotting definitions with output and input directories relocated.
source=ROOT/'py/make_figures.py'
tree=ast.parse(source.read_text(),filename=str(source))
for node in tree.body:
 if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name):
  name=node.targets[0].id
  if name in {'R','FIG'}:
   path=ROOT/'v2/runs/v2' if name=='R' else BASE/'figures'
   node.value=ast.Call(func=ast.Name(id='Path',ctx=ast.Load()),args=[ast.Constant(str(path))],keywords=[])
ast.fix_missing_locations(tree)
scope={'__name__':'saved_plot_definitions','__file__':str(source)}
exec(compile(tree,str(source),'exec'),scope)
plt=scope['plt']
original_save=scope['save']
def save(fig,name):
 axes=fig.axes
 if name=='fig1_force_laws':
  fig.set_size_inches(7,2.8)
  for ax in axes:
   for text in ax.get_legend().get_texts():text.set_fontsize(7)
  for i,ax in enumerate(axes):ax.set_xlabel(ax.get_xlabel().replace('centre','center')+'\n('+chr(97+i)+')')
  fig.tight_layout(pad=.7,w_pad=1.2)
 elif name=='fig4_dose_response':
  fig.set_size_inches(8.4,3.1)
  titles=['Prediction horizon','Stopping horizon','Minimum separation']
  units=['Horizon (s)','Stopping time (s)','Gap threshold (m)']
  for i,ax in enumerate(axes):
   if ax.get_legend():ax.get_legend().remove()
   ax.set_title(titles[i],fontsize=10,pad=8)
   ax.set_xlabel(units[i]+'\n('+chr(97+i)+')',fontsize=9)
   ax.set_ylim(0,.65)
   # Two outcome curves; the third line is the vertical parameter reference.
   ax.lines[1].set_marker('s');ax.lines[1].set_linestyle('-.')
  axes[0].set_ylabel('Fraction of trials',fontsize=9)
  handles=[axes[0].lines[0],axes[0].lines[1]]
  fig.legend(handles,['Safe goal completion','Deadlock'],loc='lower center',ncol=2,frameon=False,fontsize=9)
  fig.tight_layout(rect=(0,.12,1,1),pad=.6,w_pad=1.2)
 original_save(fig,name)
scope['save']=save
scope['fig_forces']()
scope['fig_dose']()
