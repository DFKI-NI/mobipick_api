#!/usr/bin/env python3

import warnings
from typing import Any, List, Optional, Sequence
import rospy
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Pose
from object_pose_msgs.msg import ObjectPose
from pose_selector.srv import ClassQuery, GetPoses, GetPosesResponse, PoseDelete, PoseDeleteRequest
from mobipick_api.manipulation import Manipulation
from mobipick_api.semantic_environment_rep import SemEnvRep

def _deprecated(message: str) -> None:
    rospy.logwarn(f'DEPRECATED: {message}')
    warnings.warn(message, DeprecationWarning, stacklevel=3)


class Perception:
    def __init__(self, namespace: str, arm: Manipulation, semantic_env_rep: SemEnvRep) -> None:
        self.namespace = namespace
        self.arm = arm
        self.semantic_env_rep = semantic_env_rep
        self.pose_selector_service_timeout = float(rospy.get_param(
            '~pose_selector_service_timeout', 2.0))
        self.pose_selector_activate_srv_name = rospy.get_param(
            '~pose_selector_activate_srv_name', '/pick_pose_selector_node/pose_selector_activate')
        self.pose_selector_class_query_srv_name = rospy.get_param(
            '~pose_selector_class_query_srv_name', '/pick_pose_selector_node/pose_selector_class_query')
        self.pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pose_selector_get_all_poses_srv_name', '/pick_pose_selector_node/pose_selector_get_all')
        self.pose_selector_delete_srv_name = rospy.get_param(
            '~pose_selector_delete_srv_name', '/pick_pose_selector_node/pose_selector_delete')
        self.pose_selector_clear_srv_name = rospy.get_param(
            '~pose_selector_clear_srv_name', '/pick_pose_selector_node/pose_selector_clear')
        # AnyGrasp's stand-alone open-set detector (Grounding DINO + SAM2), see
        # mobipick_graspness/nodes/segmentation_action.py. Accepted detections are
        # pushed into the pose selector by the action server itself.
        self.detect_objects_action_name = rospy.get_param(
            '~detect_objects_action_name', '/mobipick/detect_objects')
        self.detect_objects_server_timeout = float(rospy.get_param('~detect_objects_server_timeout', 2.0))
        self.detect_objects_result_timeout = float(rospy.get_param('~detect_objects_result_timeout', 120.0))
        self._detect_objects_client = None
        # Second stage of the two-stage open-set pipeline (see detect_proposals / verify_candidates).
        self.verify_objects_action_name = rospy.get_param(
            '~verify_objects_action_name', '/mobipick/verify_objects')
        self.verify_objects_result_timeout = float(rospy.get_param('~verify_objects_result_timeout', 300.0))
        # mobipick_active_perception: InspectObject (open-set close-up inspection, for testing and
        # for Perceive) and Perceive (the one perception entry point for callers)
        self.inspect_object_action_name = rospy.get_param('~inspect_object_action_name', '/mobipick/inspect_object')
        self.inspect_object_result_timeout = float(rospy.get_param('~inspect_object_result_timeout', 1800.0))
        self._inspect_object_client = None
        self.perceive_action_name = rospy.get_param('~perceive_action_name', '/mobipick/perceive')
        self.perceive_server_timeout = float(rospy.get_param('~perceive_server_timeout', 2.0))
        self.perceive_result_timeout = float(rospy.get_param('~perceive_result_timeout', 3600.0))
        self._perceive_client = None
        # arm pose of the fallback DOPE observation when the Perceive server is not running
        self.perception_observation_pose = rospy.get_param('~perception_observation_pose', 'observe100cm_right')
        self._verify_objects_client = None

        # Creating a rospy ServiceProxy does not contact the service.  Always create
        # every proxy here so that a pose selector which starts after Robot can be
        # discovered by the readiness check made immediately before each call.
        self.activate_pose_selector_srv = rospy.ServiceProxy(
            self.pose_selector_activate_srv_name, SetBool)
        self.pose_selector_class_query_srv = rospy.ServiceProxy(
            self.pose_selector_class_query_srv_name, ClassQuery)
        self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(
            self.pose_selector_get_all_poses_srv_name, GetPoses)
        self.pose_selector_clear_srv = rospy.ServiceProxy(
            self.pose_selector_clear_srv_name, Trigger)
        self.pose_selector_delete_srv = rospy.ServiceProxy(
            self.pose_selector_delete_srv_name, PoseDelete)

        rospy.loginfo(
            f'configured pose selector services: {self.pose_selector_activate_srv_name}, '
            f'{self.pose_selector_class_query_srv_name}, '
            f'{self.pose_selector_get_all_poses_srv_name}, '
            f'{self.pose_selector_clear_srv_name}, '
            f'{self.pose_selector_delete_srv_name}'
        )

    def wait_for_pose_selector_srv(self, srv_name: str) -> None:
        """Wait for one pose selector service or raise an actionable ROS error."""
        rospy.loginfo(f'waiting for pose selector service: {srv_name}')
        try:
            rospy.wait_for_service(srv_name, timeout=self.pose_selector_service_timeout)
        except rospy.ROSException as exc:
            message = (
                f"Pose selector service '{srv_name}' is unavailable after "
                f'{self.pose_selector_service_timeout:.1f}s. Ensure the pose selector '
                'node is running; this operation can be retried once it is available. '
                f'ROS error: {exc}'
            )
            rospy.logerr(message)
            raise rospy.ServiceException(message) from exc

    def _call_pose_selector_service(self, srv_name: str, service_proxy, *args):
        self.wait_for_pose_selector_srv(srv_name)
        try:
            return service_proxy(*args)
        except rospy.ServiceException as exc:
            message = (
                f"Pose selector service '{srv_name}' became unavailable while being "
                'called. Check the pose selector node and retry the operation. '
                f'ROS error: {exc}'
            )
            rospy.logerr(message)
            raise rospy.ServiceException(message) from exc

    # ------------------------------------------------------------------ perceive
    def perceive(self, targets: Optional[Sequence[str]] = None, confidence: str = 'high', *,
                 dope_only: bool = False, gd_only: bool = False, use_vlm: bool = True, align: bool = True,
                 observation_pose: Optional[str] = None, session_dir: str = '',
                 observation_list: Optional[List[str]] = None, feedback_cb: Optional[Any] = None) -> Optional[str]:
        '''Perceive ``targets`` and return one human readable text with a line per target.

        A thin client of the ``mobipick_active_perception`` Perceive action (``/mobipick/perceive``),
        which does the work: ``targets`` mix known objects (``multimeter``, ``multimeter_1``: DOPE),
        open-set descriptions (``tennis ball``: Grounding DINO) and tables (``table_2``);
        ``confidence`` (default ``high``) matters for open-set targets only (``low``: one detector
        view, nothing committed; ``high``: close-up inspection, VLM verification, committed pose).
        ``dope_only`` / ``gd_only`` (not both), ``use_vlm``, ``align`` and ``observation_pose`` are
        passed through. The text starts with ``success:``, ``partial:`` or ``failed:``.

        Without the Perceive server the old closed-set observation runs instead (deprecated) and
        only targets that are pose selector ids can be confirmed.

        Legacy: ``perceive()`` / ``perceive(observation_list=[...])`` without ``targets`` keeps the
        old behaviour (DOPE from each arm pose, nothing reported) and warns.
        '''
        if targets is None:
            _deprecated('perceive() without targets is the old closed-set observation; use '
                        'perceive(targets=[...], confidence=...) for per-target results')
            return self._legacy_perceive(observation_list)
        if isinstance(targets, str):
            targets = [targets]
        targets = [str(t).strip() for t in targets if str(t).strip()]
        if observation_list:
            _deprecated('observation_list is ignored when targets are given; use observation_pose')
        client = self._perceive_action_client()
        if client is None:
            _deprecated(f'the active perception pipeline ({self.perceive_action_name}, mobipick_active_perception) '
                        'is not running: perceive falls back to the old closed-set observation')
            return self._perceive_without_pipeline(targets, observation_pose)
        from mobipick_active_perception.msg import PerceiveGoal

        goal = PerceiveGoal(targets=targets, confidence=confidence or 'high', dope_only=bool(dope_only),
                            gd_only=bool(gd_only), skip_vlm=not use_vlm, skip_alignment=not align,
                            observation_pose=observation_pose or '', session_dir=session_dir or '')
        rospy.loginfo(f'perceive {targets} with {goal.confidence} confidence')
        client.send_goal(goal, feedback_cb=feedback_cb or (lambda f: rospy.loginfo(f'perceive: {f.stage}')))
        if not client.wait_for_result(rospy.Duration(self.perceive_result_timeout)):
            client.cancel_goal()
            return self._failed(targets, f'the perceive action gave no result within {self.perceive_result_timeout:.0f} s')
        result = client.get_result()
        if result is None or not result.summary:
            return self._failed(targets, f'the perceive action ended without a result ({client.get_goal_status_text()})')
        (rospy.loginfo if result.success else rospy.logwarn)(f'perceive:\n{result.summary}')
        return result.summary

    @staticmethod
    def _failed(targets: Sequence[str], reason: str) -> str:
        rospy.logerr(reason)
        return '\n'.join(['failed:'] + [f'{t}: failed, {reason}' for t in targets])

    def _perceive_action_client(self):
        '''The Perceive action client once its server answers, else None (package or server missing).'''
        try:
            import actionlib
            from mobipick_active_perception.msg import PerceiveAction
        except ImportError as exc:
            rospy.logwarn(f'mobipick_active_perception Perceive messages unavailable ({exc})')
            return None
        if self._perceive_client is None:
            self._perceive_client = actionlib.SimpleActionClient(self.perceive_action_name, PerceiveAction)
        if not self._perceive_client.wait_for_server(rospy.Duration(self.perceive_server_timeout)):
            return None
        return self._perceive_client

    def _perceive_without_pipeline(self, targets: Sequence[str], observation_pose: Optional[str]) -> str:
        '''Old closed-set observation, then a pose selector lookup per target.'''
        self._legacy_perceive([observation_pose or self.perception_observation_pose])
        lines = []
        for target in targets:
            if self.is_object_inside_pose_selector(target):
                lines.append(f'{target}: perceived with DOPE ({target} is in the pose selector)')
            else:
                lines.append(f'{target}: failed, not verified: only a plain DOPE observation was made because the '
                             'active perception pipeline (mobipick_active_perception perceive server) is not running')
        found = sum(1 for line in lines if ': perceived' in line)
        status = 'success' if found == len(lines) else ('failed' if found == 0 else 'partial')
        return '\n'.join([f'{status}:'] + lines)

    def _legacy_perceive(self, observation_list: Optional[List[str]]) -> None:
        if not observation_list:
            return self._perceive_without_moving_arm()
        rospy.loginfo(f'perceiving objects, observation_list : {observation_list}')
        # iterate over arm observation poses and enable/disable pose selector
        for observation_pose in observation_list:
            rospy.loginfo(f'moving arm to {observation_pose}')
            # move arm to observation pose
            self.arm.move(observation_pose)
            self._perceive_without_moving_arm()

    def _perceive_without_moving_arm(self) -> None:
        rospy.loginfo(f'perceiving objects')
        # activate pose selector
        self.set_closed_set_perception(True)
        # wait until pose selector gets updates
        rospy.sleep(5.0)
        # deactivate pose selector detections
        self.set_closed_set_perception(False)

    def set_closed_set_perception(self, active: bool) -> None:
        '''Activate / deactivate the pose selector's recording of DOPE detections.'''
        rospy.loginfo(f'{"activating" if active else "deactivating"} pose selector')
        resp = self._call_pose_selector_service(
            self.pose_selector_activate_srv_name, self.activate_pose_selector_srv, bool(active))
        rospy.loginfo(f'pose selector response to {"activation" if active else "de-activation"} request: {resp}')

    def detect_open_set(self, object_name: str, use_vlm_verifier: bool = False,
                        observation_pose: Optional[str] = None, box_threshold: float = 0.0,
                        text_threshold: float = 0.0, accept_threshold: float = 0.0) -> Optional[Any]:
        '''Run AnyGrasp's open-set detector for a free-form description ("coke can") on
        the current camera view, optionally after moving the arm to ``observation_pose``.
        Thresholds left at 0 use the detector node's dynamic_reconfigure defaults.

        Returns the ``grasplan/DetectObjectsResult`` (``success``, ``message``,
        ``detections`` with 2-D boxes/masks, ``boxes`` with map-frame oriented 3-D
        boxes, ``objects`` accepted into the pose selector as ``<class_id>_<n>``), or
        None when the action server is unavailable or timed out.
        '''
        # imported here so the module stays importable in ROS-master-free unit tests
        import actionlib
        from actionlib_msgs.msg import GoalStatus
        from grasplan.msg import DetectObjectsAction, DetectObjectsGoal

        if observation_pose:
            rospy.loginfo(f'moving arm to {observation_pose} before open-set detection')
            self.arm.move(observation_pose)
        if self._detect_objects_client is None:
            self._detect_objects_client = actionlib.SimpleActionClient(
                self.detect_objects_action_name, DetectObjectsAction)
        client = self._detect_objects_client
        if not client.wait_for_server(rospy.Duration(self.detect_objects_server_timeout)):
            rospy.logerr(
                f'open-set detection action server {self.detect_objects_action_name} is unavailable '
                f'after {self.detect_objects_server_timeout:.1f}s; is AnyGrasp running?')
            return None
        rospy.loginfo(f'open-set detection of {object_name!r} (vlm verifier: {use_vlm_verifier})')
        client.send_goal(
            DetectObjectsGoal(object_name=object_name, use_vlm_verifier=use_vlm_verifier,
                              box_threshold=box_threshold, text_threshold=text_threshold,
                              accept_threshold=accept_threshold),
            feedback_cb=lambda feedback: rospy.loginfo(f'open-set detection: {feedback.stage}'))
        if not client.wait_for_result(rospy.Duration(self.detect_objects_result_timeout)):
            client.cancel_goal()
            rospy.logerr(f'open-set detection of {object_name!r} timed out after '
                         f'{self.detect_objects_result_timeout:.1f}s')
            return None
        result = client.get_result()
        state = client.get_state()
        if state != GoalStatus.SUCCEEDED or result is None or not result.success:
            rospy.logwarn(f'open-set detection of {object_name!r} did not succeed (state {state}): '
                          f'{getattr(result, "message", client.get_goal_status_text())}')
        else:
            rospy.loginfo(f'open-set detection: {result.message}')
        return result

    def _run_action(self, client, goal, name: str, result_timeout: float) -> Optional[Any]:
        from actionlib_msgs.msg import GoalStatus

        if not client.wait_for_server(rospy.Duration(self.detect_objects_server_timeout)):
            rospy.logerr(f'{name} action server is unavailable after '
                         f'{self.detect_objects_server_timeout:.1f}s; is AnyGrasp running?')
            return None
        client.send_goal(goal, feedback_cb=lambda feedback: rospy.loginfo(f'{name}: {feedback.stage}'))
        if not client.wait_for_result(rospy.Duration(result_timeout)):
            client.cancel_goal()
            rospy.logerr(f'{name} timed out after {result_timeout:.1f}s')
            return None
        result = client.get_result()
        state = client.get_state()
        if state != GoalStatus.SUCCEEDED or result is None or not result.success:
            rospy.logwarn(f'{name} did not succeed (state {state}): '
                          f'{getattr(result, "message", client.get_goal_status_text())}')
        else:
            rospy.loginfo(f'{name}: {result.message}')
        return result

    def detect_proposals(self, object_name: str, query_variants: Optional[List[str]] = None,
                         max_proposals: int = 3, observation_pose: Optional[str] = None,
                         box_threshold: float = 0.0, text_threshold: float = 0.0,
                         return_frame: bool = False) -> Optional[Any]:
        '''First stage of two-stage open-set perception: capture the current view (optionally
        after moving the arm to ``observation_pose``), run Grounding DINO for ``object_name``
        (plus any ``query_variants``, one detector pass) and return the best ``max_proposals``
        proposals without accepting anything. The result carries ``view_id``, ``camera_pose``
        and per proposal ``detection_id``, ``score``, ``bbox_xyxy`` and a coarse map-frame
        ``position``; the node keeps the frame so :meth:`verify_candidates` can build the
        evidence later. Returns the ``grasplan/DetectObjectsResult`` or None.
        '''
        import actionlib
        from grasplan.msg import DetectObjectsAction, DetectObjectsGoal

        if observation_pose:
            rospy.loginfo(f'moving arm to {observation_pose} before proposing {object_name!r}')
            self.arm.move(observation_pose)
        if self._detect_objects_client is None:
            self._detect_objects_client = actionlib.SimpleActionClient(
                self.detect_objects_action_name, DetectObjectsAction)
        goal = DetectObjectsGoal(object_name=object_name, query_variants=list(query_variants or []),
                                 box_threshold=box_threshold, text_threshold=text_threshold,
                                 max_proposals=max_proposals, proposals_only=True,
                                 return_frame=return_frame)
        rospy.loginfo(f'open-set proposals for {object_name!r} (top {max_proposals})')
        return self._run_action(self._detect_objects_client, goal, 'open-set proposals',
                                self.detect_objects_result_timeout)

    def verify_candidates(self, object_name: str, candidates: List[List[str]],
                          query_variants: Optional[List[str]] = None, include_full_frames: bool = True,
                          commit: bool = True) -> Optional[Any]:
        '''Second stage: verify candidate objects, each given as the list of ``detection_id``s
        (from one or several :meth:`detect_proposals` views) believed to show the same physical
        object. Every candidate is judged by one multi-image VLM request; the result holds one
        ``grasplan/Verdict`` per candidate whose ``status`` separates MATCH / NO_MATCH from
        verifier failures (TIMEOUT, HTTP_ERROR, NO_CONTENT, BAD_JSON, UNAVAILABLE, ERROR).
        With ``commit`` accepted candidates are pushed to the pose selector. Returns the
        ``grasplan/VerifyObjectsResult`` or None.
        '''
        import actionlib
        from grasplan.msg import CandidateGroup, VerifyObjectsAction, VerifyObjectsGoal

        if self._verify_objects_client is None:
            self._verify_objects_client = actionlib.SimpleActionClient(
                self.verify_objects_action_name, VerifyObjectsAction)
        goal = VerifyObjectsGoal(object_name=object_name, query_variants=list(query_variants or []),
                                 candidates=[CandidateGroup(detection_ids=list(ids)) for ids in candidates],
                                 include_full_frames=include_full_frames, commit=commit)
        rospy.loginfo(f'verifying {len(candidates)} candidate(s) of {object_name!r} with the VLM')
        return self._run_action(self._verify_objects_client, goal, 'open-set verification',
                                self.verify_objects_result_timeout)

    def inspect_object(self, queries: List[Any], table: str = '', align: bool = True, align_to: str = '',
                       use_vlm: bool = True, session_dir: str = '', config_overrides: str = '',
                       feedback_cb: Optional[Any] = None) -> Optional[Any]:
        '''Close-up open-set inspection by the ``mobipick_active_perception`` InspectObject server,
        for the table the robot already stands at: observation view, Grounding DINO proposals,
        close-ups of the candidates that need one (viewpoints sampled around them, MoveIt), and
        commit to the pose selector. With ``use_vlm`` (default) every candidate is verified by the
        VLM and only confirmed MATCHes are committed; without it the Grounding DINO detections of
        the close-up view are committed. ``queries`` are names or ``{"name", "variants"}`` dicts;
        ``table`` only labels the recorded session. With ``align`` and DISC running, the base is
        first driven along its heading so the observation view is centred on the DISC position of
        ``align_to`` (default: the first query DISC knows nearby). Returns the
        ``InspectObjectResult`` (``accepted`` ids, ``accepted_query``, map-frame ``poses``,
        ``outcomes`` per query, ``session_dir``, ``alignment_cm``) or None.

        Callers should use :meth:`perceive` (``confidence="high"``); this is the internal step,
        exposed for testing (RQT) and data collection.
        '''
        import actionlib
        from mobipick_active_perception.msg import InspectObjectAction, InspectObjectGoal, Query

        if self._inspect_object_client is None:
            self._inspect_object_client = actionlib.SimpleActionClient(self.inspect_object_action_name,
                                                                       InspectObjectAction)
        goal = InspectObjectGoal(table=table, align=bool(align), align_to=align_to, skip_vlm=not use_vlm,
                                 session_dir=session_dir, config_overrides=config_overrides)
        for query in queries:
            if isinstance(query, str):
                goal.queries.append(Query(name=query))
            else:
                goal.queries.append(Query(name=str(query['name']), variants=list(query.get('variants', []) or [])))
        rospy.loginfo(f'inspecting {[q.name for q in goal.queries]}'
                      f'{" (aligned with DISC)" if align else ""}{"" if use_vlm else " without VLM"}')
        if not self._inspect_object_client.wait_for_server(rospy.Duration(self.detect_objects_server_timeout)):
            rospy.logerr(f'action server {self.inspect_object_action_name} not available')
            return None
        self._inspect_object_client.send_goal(goal, feedback_cb=feedback_cb)
        if not self._inspect_object_client.wait_for_result(rospy.Duration(self.inspect_object_result_timeout)):
            rospy.logerr(f'{self.inspect_object_action_name}: no result within '
                         f'{self.inspect_object_result_timeout:.0f}s')
            self._inspect_object_client.cancel_goal()
            return None
        result = self._inspect_object_client.get_result()
        (rospy.loginfo if result.success else rospy.logerr)(f'inspect_object: {result.message}')
        return result

    def inspect_table(self, queries: List[Any], table: str = '', align: bool = True, align_to: str = '',
                      session_dir: str = '', config_overrides: str = '',
                      feedback_cb: Optional[Any] = None) -> Optional[Any]:
        '''Deprecated name of :meth:`inspect_object` (the robot is assumed to be at the table).'''
        _deprecated('inspect_table was renamed to inspect_object; callers should use perceive(..., confidence="high")')
        return self.inspect_object(queries, table=table, align=align, align_to=align_to, session_dir=session_dir,
                                   config_overrides=config_overrides, feedback_cb=feedback_cb)

    def clear_poses_for_table(self, table: str) -> None:
        # get current facts
        facts = self.semantic_env_rep.get_facts()
        # clear facts for table from pose_selector
        for fact in facts:
            if fact.name == "on" and fact.values[1] == table:
                class_id, instance_id = fact.values[0].rsplit("_", 1)
                request = PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id))
                self._call_pose_selector_service(
                    self.pose_selector_delete_srv_name,
                    self.pose_selector_delete_srv,
                    request)

    def is_object_inside_pose_selector(self, object_of_interest: str) -> bool:
        resp: GetPosesResponse = self._call_pose_selector_service(
            self.pose_selector_get_all_poses_srv_name,
            self.pose_selector_get_all_poses_srv)
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                assert isinstance(pose_selector_object, ObjectPose)
                anchored_object = str(pose_selector_object.class_id) + '_' + str(pose_selector_object.instance_id)
                if anchored_object == object_of_interest:
                    rospy.loginfo(f'object {object_of_interest} found!')
                    return True
                else:
                    rospy.logdebug(f'object {anchored_object} not equal to {object_of_interest}, trying next object')
            rospy.logerr(f'tried all objects without success, {object_of_interest} could not be found in pose selector')
            return False
        else:
            rospy.logwarn('pose selector is empty')
        return False

    def get_object_pose(self, object_name) -> Optional[Pose]:
        resp: GetPosesResponse = self._call_pose_selector_service(
            self.pose_selector_get_all_poses_srv_name,
            self.pose_selector_get_all_poses_srv)
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                assert isinstance(pose_selector_object, ObjectPose)
                anchored_object = str(pose_selector_object.class_id) + '_' + str(pose_selector_object.instance_id)
                if anchored_object == object_name:
                    rospy.loginfo(f'object {object_name} found!')
                    return pose_selector_object.pose
                else:
                    rospy.logdebug(f'object {anchored_object} not equal to {object_name}, trying next object')
            rospy.logerr(f'tried all objects without success, {object_name} could not be found in pose selector')
            return None
        else:
            rospy.logwarn('pose selector is empty')
        return None
