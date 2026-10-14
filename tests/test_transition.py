import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from g1_action_studio.collision import CollisionModel, find_path, smooth_path
from g1_action_studio.transition import plan_transition
from g1_action_studio.trajectory import metrics


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / 'examples/transition'


class TransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.model = CollisionModel()
        except ImportError as error:
            raise unittest.SkipTest(str(error))
        cls.a = json.loads((EXAMPLES/'action-a.json').read_text())['samples'][-1]['q']
        cls.b = json.loads((EXAMPLES/'action-b.json').read_text())['samples'][0]['q']

    def test_endpoints_clear_but_direct_path_penetrates_body(self):
        self.assertTrue(self.model.clearance(self.a)['valid'])
        self.assertTrue(self.model.clearance(self.b)['valid'])
        middle = [a + .3*(b-a) for a,b in zip(self.a,self.b)]
        collision = self.model.clearance(middle)
        self.assertFalse(collision['valid'])
        self.assertLess(collision['distance_m'], 0)
        self.assertIn('pelvis_collision', collision['pair'])
        self.assertFalse(self.model.edge_clear(self.a,self.b,.02))

    def test_interval_check_finds_collision_away_from_midpoint(self):
        midpoint=[(a+b)/2 for a,b in zip(self.a,self.b)]
        self.assertTrue(self.model.clearance(midpoint,.01)['valid'])
        self.assertFalse(self.model.edge_clear(self.a,self.b,.01))

    def test_right_arm_can_detour_with_left_arm_fixed(self):
        def mirror(q):
            signs=[1,-1,-1,1,-1,1,-1]
            return [v*s for v,s in zip(q[7:],signs)]+[v*s for v,s in zip(q[:7],signs)]
        a,b=mirror(self.a),mirror(self.b)
        path=find_path(self.model,a,b,'right',seed=5)
        self.assertGreater(len(path),2)
        for point in path:self.assertEqual(point[:7],a[:7])
        for start,end in zip(path,path[1:]):self.assertTrue(self.model.edge_clear(start,end,.02))

    def test_search_detours_and_keeps_other_arm_fixed(self):
        path = find_path(self.model,self.a,self.b,'left',seed=5)
        self.assertGreater(len(path),2)
        self.assertEqual(path[0],self.a)
        self.assertEqual(path[-1],self.b)
        for a,b in zip(path,path[1:]):
            self.assertEqual(a[7:],self.a[7:])
            self.assertTrue(self.model.edge_clear(a,b,.02))
        frames = smooth_path(path,.5,2)
        values = metrics(frames)
        self.assertLessEqual(values['max_abs_velocity_rad_s'],.5)
        self.assertLessEqual(values['max_abs_acceleration_rad_s2'],2)
        self.assertLess(values['first_step_max_velocity_rad_s'],.001)
        self.assertLess(values['last_step_max_velocity_rad_s'],.001)

    def test_clear_direct_path_needs_no_search(self):
        end=list(self.b);end[0]-=.03
        self.assertEqual(find_path(self.model,self.b,end,'left',iterations=0),[self.b,end])

    def test_invalid_endpoints_and_exhausted_search_are_rejected(self):
        bad=list(self.a);bad[0]=float('nan')
        with self.assertRaises(ValueError):find_path(self.model,bad,self.b,'left')
        bad=list(self.a);bad[0]=10
        with self.assertRaises(ValueError):find_path(self.model,bad,self.b,'left')
        with self.assertRaisesRegex(ValueError,'预算'):find_path(self.model,self.a,self.b,'left',iterations=0)
        with self.assertRaises(ValueError):smooth_path([self.a,self.b],float('nan'),2)
        with self.assertRaises(ValueError):CollisionModel([0]*14)

    def test_finger_envelopes_and_both_arms_are_included(self):
        m=self.model.model
        names=[m.geom(i).name for i in range(m.ngeom)]
        self.assertEqual(sum(n.endswith('_sweep') for n in names),6)
        self.assertIn('head_link_collision',names)
        self.assertIn('left_hand_palm_link_collision',names)
        self.assertIn('right_hand_palm_link_collision',names)
        cross_arm=[]
        for (a,b),mount in zip(self.model.pairs,self.model.mount_pairs):
            na,nb=m.geom(int(a)).name,m.geom(int(b)).name
            if na.startswith('left_') and nb.startswith('right_'):
                cross_arm.append((na,nb))
                self.assertFalse(mount)
        self.assertIn(('left_hand_thumb_sweep','right_hand_thumb_sweep'),cross_arm)

    def test_complete_transition_outputs_uniform_loadable_action(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            output=Path(directory)/'plan.json'
            with patch.dict('sys.modules',{'unitree_sdk2py':None}):
                result=plan_transition(EXAMPLES/'action-a.json',EXAMPLES/'action-b.json',output,seed=5)
            self.assertEqual(result['kind'], 'sequence')
            self.assertEqual(result['status'],'offline_collision_pass')
            self.assertGreaterEqual(result['minimum_clearance_m'],.02)
            plan=json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual({f['phase'] for f in plan['frames']},{'action_1','transition','action_2'})
            self.assertEqual(plan['frames'][0]['q'][7:],plan['frames'][-1]['q'][7:])
            from g1_action_studio.author import load_action
            self.assertEqual(load_action(output), plan)
            with self.assertRaises(FileExistsError):
                plan_transition(EXAMPLES/'action-a.json',EXAMPLES/'action-b.json',output)

    def test_collision_inside_action_is_rejected_without_output(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            source=Path(directory)/'bad.json';output=Path(directory)/'plan.json'
            data=json.loads((EXAMPLES/'action-a.json').read_text())
            collision=[a+.3*(b-a) for a,b in zip(self.a,self.b)]
            for row in data['samples'][10:40]:row['q']=collision
            source.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'完整动作'):
                plan_transition(source,EXAMPLES/'action-b.json',output,seed=5,
                                max_velocity=1000,max_acceleration=1e6)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
