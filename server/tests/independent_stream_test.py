"""Independent acquisition rates, delayed delivery and dropout over real ROS2."""
import asyncio
import copy
import json
import math
import struct
import unittest
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from helpers import RosFixture, config, jpeg, arm_state

class IndependentStreamTest(unittest.IsolatedAsyncioTestCase):
    async def test_independent_rates_jitter_and_dropout_recover_with_new_evidence(self):
        c=config('minimal.json')
        c.cameras[0].sync_tolerance_sec=.03
        c.cameras[0].camera_info_mode='ros_rectified'
        f=RosFixture(c)
        self.addCleanup(f.close)
        await f.ready()
        f.driver.timer.cancel()
        enabled={'depth':True}
        tasks=[]
        angle=.37
        pose=[.17,-.23,.41]
        def header(message, frame='overhead_optical'):
            message.header.frame_id=frame
            message.header.stamp=f.driver.get_clock().now().to_msg()
            return message
        async def states():
            i=0
            while True:
                sample=dict(arm_state(),stamp_ns=f.driver.get_clock().now().nanoseconds)
                f.driver.state_publishers['arm'].publish(String(data=json.dumps(sample)))
                await asyncio.sleep(.021+(i%3)*.002)
                i+=1
        async def rgb():
            i=0
            while True:
                message=header(CompressedImage(format='jpeg',data=jpeg()))
                f.driver.image_publishers['overhead'].publish(message)
                info=CameraInfo(header=copy.deepcopy(message.header),width=48,height=32,
                    k=[100.,0.,24.,0.,100.,16.,0.,0.,1.],d=[.1,-.01],
                    r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                    p=[113.7,0.,23.3,0.,0.,107.9,15.8,0.,0.,0.,1.,0.])
                f.driver.info_publishers['overhead'].publish(info)
                await asyncio.sleep(.031+(i%3)*.003)
                i+=1
        async def depth():
            i=0
            while True:
                message=header(Image(width=48,height=32,encoding='32FC1',step=192,
                    data=struct.pack('<1536f', *[1.234+(j%3-1)*.002 for j in range(1536)])))
                # Acquisition and delivery times differ, independently of RGB.
                await asyncio.sleep(.012+(i%2)*.008)
                if enabled['depth']: f.driver.depth_publishers['overhead'].publish(message)
                await asyncio.sleep(.029)
                i+=1
        async def transforms():
            while True:
                tf=header(TransformStamped(), 'world')
                tf.child_frame_id='overhead_optical'
                tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=pose
                tf.transform.rotation.z=math.sin(angle/2)
                tf.transform.rotation.w=math.cos(angle/2)
                f.driver.tf_broadcaster.sendTransform(tf)
                await asyncio.sleep(.011)
        tasks=[asyncio.create_task(fn()) for fn in (states,rgb,depth,transforms)]
        try:
            # Require captures after the independent publishers have taken over.
            await asyncio.sleep(.08)
            captures=[]
            for _ in range(3):
                reply=await f.gateway.request('capture','overhead',{},1.)
                self.assertEqual(200,reply.status,reply.payload)
                captures.append(reply.payload)
                self.assertLessEqual(abs(reply.payload['sync_delta_ns']),30_000_000)
                point=f.robot.pixels.project(reply.payload['capture_id'],[24,16])['position']
                z=1.232
                x,y=(24-23.3)*z/113.7,(16-15.8)*z/107.9
                expected=[pose[0]+math.cos(angle)*x-math.sin(angle)*y,
                          pose[1]+math.sin(angle)*x+math.cos(angle)*y,pose[2]+z]
                for a,b in zip(point,expected):self.assertAlmostEqual(a,b,delta=1e-6)
            self.assertTrue(any(c['sync_delta_ns'] != 0 for c in captures))
            enabled['depth']=False
            missing=await f.gateway.request('capture','overhead',{},.15)
            self.assertEqual(504,missing.status)
            enabled['depth']=True
            restored=await f.gateway.request('capture','overhead',{},1.)
            self.assertEqual(200,restored.status,restored.payload)
            self.assertNotIn(restored.payload['capture_id'],[c['capture_id'] for c in captures])
            self.assertFalse(f.driver.calls)
        finally:
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)

    async def test_concurrent_fixtures_have_distinct_domains(self):
        first=RosFixture(config('minimal.json'))
        self.addCleanup(first.close)
        second=RosFixture(config('minimal.json'))
        self.addCleanup(second.close)
        await asyncio.gather(first.ready(),second.ready())
        self.assertNotEqual(first.context.get_domain_id(),second.context.get_domain_id())
