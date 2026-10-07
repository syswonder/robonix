# SPDX-License-Identifier: MulanPSL-2.0
"""Check saved execution identity, order, and streaming timing evidence."""
import json
import os
import pathlib
import re
from datetime import datetime
OUT = pathlib.Path(os.environ['RTDL_TEST_OUT']).resolve()

def records(name):
    """Load structured component logs and attach sortable timestamps."""
    rows=[]
    for line in (OUT/f'{name}.log').read_text().splitlines():
        d=json.loads(line)
        d['t']=datetime.fromisoformat(d['ts'][:26]).timestamp()
        rows.append(d)
    return rows

pilot=records('pilot'); executor=records('executor')
summaries=sorted(OUT.glob('count-*.summary.json'))
if len(summaries)!=11:
    raise SystemExit('Expected all 11 count summaries (0..10)')
all_passed=True
for path in summaries:
    s=json.loads(path.read_text())
    events=[json.loads(l) for l in path.with_name(path.name.replace('.summary.json','.jsonl')).read_text().splitlines()]
    plans=[e['event']['plan'] for e in events if 'plan' in e['event']]
    details=[]
    for p in plans:
        calls=[x['call'] for x in p['nodes'] if x.get('node_kind')==2]
        ids=[c['call_id'] for c in calls]
        dispatched=[r for r in executor if s['start']<=r['t']<=s['end'] and any(f'call_id={cid} ' in r['msg'] for cid in ids)]
        full=next(r for r in pilot if s['start']<=r['t']<=s['end'] and f'-- plan start plan_id={p["plan_id"]} ' in r['msg'])
        raw=[r for r in pilot if '[pilot/rtdl/raw] raw_content=' in r['msg'] and s['start']<=r['t']<=full['t']]
        if not raw:
            raise SystemExit('Missing raw_content debug log; streaming timing is unverified')
        end=raw[-1]['t']
        actual=[re.search(r'call_id=(\S+)',r['msg'])[1] for r in dispatched]
        details.append({'plan_id':p['plan_id'],'calls':ids,'dispatch_count':len(actual),'ordered_once':actual==ids,'first_dispatch_before_generation_end_s':round(end-dispatched[0]['t'],4) if dispatched else None,'calls_dispatched_before_generation_end':sum(r['t']<end for r in dispatched)})
    s['execution_checks']=details
    s['streaming_pass']=s['pass'] and all(d['ordered_once'] and d['first_dispatch_before_generation_end_s'] is not None and d['first_dispatch_before_generation_end_s']>0 for d in details)
    all_passed=all_passed and s['streaming_pass']
    path.write_text(json.dumps(s,ensure_ascii=False,indent=2))
    print(s['n'],s['pass'],details)

if not all_passed:
    raise SystemExit('Execution or streaming timing checks failed; inspect the summaries')
