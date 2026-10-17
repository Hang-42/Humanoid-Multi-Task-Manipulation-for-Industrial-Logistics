"""Offline safety regressions for the new vendor timed-output registration."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from arm_ack import TIMED_CONTROLLER, require_inactive_mpc_route


class ControlPublisherTests(unittest.TestCase):
    def test_only_fresh_trusted_mpc_feedback_is_accepted(self):
        require_inactive_mpc_route((10.,0.,'/nodelet_manager'),10.1)
        for sample in (None, (9.,0.,'/nodelet_manager'),
                       (11.,0.,'/nodelet_manager'),
                       (10.,1.,'/nodelet_manager'),
                       (10.,float('nan'),'/nodelet_manager'),
                       (10.,0.,'/other')):
            with self.subTest(sample=sample), self.assertRaises(RuntimeError):
                require_inactive_mpc_route(sample,10.1)

    def check(self, nodes, route=(10.,0.,'/nodelet_manager'), uri='http://lower:1/'):
        def require(ok, message):
            if not ok:
                raise RuntimeError(message)
        master=NS(getSystemState=lambda:([('/kuavo_arm_traj',nodes)],[],[]),
                  lookupNode=lambda name:uri)
        scope=dict(rosgraph=NS(Master=lambda name:master),
                   rospy=NS(get_name=lambda:'/test_bridge'),
                   time=NS(monotonic=lambda:10.1),require=require,
                   TIMED_CONTROLLER=TIMED_CONTROLLER,
                   require_inactive_mpc_route=require_inactive_mpc_route)
        tree=ast.parse((Path(__file__).parent/'robot_bridge.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveRobot')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='check_publishers')
        exec(compile(ast.Module(body=[method],type_ignores=[]),'robot_bridge.py','exec'),scope)
        bridge=NS(args=NS(execute=True),timed_controller_uri='http://lower:1/',
                  arm_route_feedback=route)
        scope['check_publishers'](bridge)
        return bridge

    def test_required_timed_node_is_not_stopped_or_treated_as_competitor(self):
        self.assertTrue(self.check([TIMED_CONTROLLER]).timed_arm_output_registered)

    def test_other_publishers_are_still_rejected(self):
        for nodes in ([TIMED_CONTROLLER,'/other'],['/other']):
            with self.subTest(nodes=nodes), self.assertRaisesRegex(RuntimeError,'Other control publisher'):
                self.check(nodes)

    def test_changed_uri_or_rl_or_stale_feedback_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'identity changed'):
            self.check([TIMED_CONTROLLER],uri='http://lower:2/')
        for route in ((10.,1.,'/nodelet_manager'),(8.,0.,'/nodelet_manager'),None):
            with self.subTest(route=route), self.assertRaises(RuntimeError):
                self.check([TIMED_CONTROLLER],route=route)


if __name__=='__main__':
    unittest.main()
