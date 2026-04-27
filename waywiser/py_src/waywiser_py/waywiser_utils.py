#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from typing import Union

from geometry_msgs.msg import PoseStamped
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown
import psutil
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import yagmail

import yaml

RELIABLE_TRANSIENT_LOCAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def shutdown_on_process_error(event, _context):
    """Shutdown a launch when a process exits with a non-zero status."""
    returncode = getattr(event, 'returncode', 0)
    if returncode == 0:
        return []

    action = getattr(event, 'action', None)
    action_name = getattr(action, 'name', None) or getattr(action, '__class__', type(action)).__name__
    reason = f"Shutting down launch because '{action_name}' exited with code {returncode}."

    return [
        LogInfo(msg=reason),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


class FileUtils:
    """Utility functions for file and path operations."""

    @staticmethod
    def get_full_file_path(file_path, package_relative_path=''):
        if file_path == '':
            return ''

        if file_path.startswith('~'):
            file_path = os.path.expanduser(file_path)
        file_path = os.path.expandvars(file_path)
        if not file_path.startswith('/') and package_relative_path != '':
            file_path = os.path.join(
                package_relative_path,
                file_path,
            )
        file_path = os.path.abspath(file_path)

        return file_path


class NotificationUtils:
    """Utility functions for sending notifications."""

    @staticmethod
    def send_email(subject, body, email_recipient=None):
        """Send email using credentials from .env file."""
        # Get credentials from .env
        email_user = os.getenv('EMAIL_USER')
        email_pass = os.getenv('EMAIL_PASSWORD')
        default_recipient = os.getenv('EMAIL_RECIPIENT')
        smtp_server = os.getenv('SMTP_SERVER', 'smtp.gmail.com')
        smtp_port = int(os.getenv('SMTP_PORT', 587))

        # Use provided recipient or fallback to .env default
        recipient = email_recipient if email_recipient is not None else default_recipient

        # Validate credentials
        if not all([email_user, email_pass, recipient]):
            print('Email credentials not found. Check your .env file.', flush=True)
            return

        try:
            # Initialize SMTP connection
            yag = yagmail.SMTP(
                user=email_user,
                password=email_pass,
                host=smtp_server,
                port=smtp_port,
            )

            # Send email
            yag.send(to=recipient, subject=subject, contents=body)
            print('Email sent successfully!', flush=True)
        except Exception as e:
            print(f'Error sending email: {e}', flush=True)


class ProcessUtils:
    """Utility functions for managing subprocesses."""

    @staticmethod
    def create_subprocess(
        node,
        command,
        subprocess_name,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=False,
    ):
        subprocess_ = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=stdout,
            stderr=stderr,
            text=text,
        )

        node.get_logger().info(f'Started {subprocess_name} with PID [{subprocess_.pid}].')

        return subprocess_

    @staticmethod
    def cleanup_subprocesses(subprocesses):
        """Cleanup subprocesses."""
        for id_ in list(subprocesses.keys()):
            try:
                process = subprocesses.pop(id_)
                ProcessUtils.terminate_subprocess(process)
            except Exception as e:
                print(f'Error cleaning up subprocess {id_}: {e}', flush=True)

    @staticmethod
    def terminate_subprocess(process):
        """Force terminate a process and its children using psutil."""

        def _get_running_processes(processes):
            """Get list of processes that are still running."""
            running = []
            for proc in processes:
                try:
                    if proc.is_running() and proc.status() != psutil.STATUS_ZOMBIE:
                        running.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            return running

        def _send_signal_to_processes(processes, sig):
            """Send signal to all processes, handling errors gracefully."""
            for proc in processes:
                try:
                    if proc.is_running():
                        proc.send_signal(sig)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                except Exception as e:
                    print(f'Error sending signal {sig} to process {proc.pid}: {e}', flush=True)

        def _wait_for_termination(processes, timeout=10):
            """Wait for all processes to terminate within timeout."""
            if not processes:
                return True

            start_time = time.time()
            check_interval = 0.1

            while time.time() - start_time < timeout:
                running_processes = _get_running_processes(processes)
                if not running_processes:
                    return True
                time.sleep(check_interval)

            return False

        if not process or process.poll() is not None:
            print(f'Process {process.pid} is not running', flush=True)
            return True
        try:
            main_process = psutil.Process(process.pid)
            if not main_process.is_running() or main_process.status() == psutil.STATUS_ZOMBIE:
                print(f'Process {process.pid} already terminated or is zombie', flush=True)
                # Clean up zombie by calling wait() on the subprocess
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                return True

            # Get all children before starting termination
            children = main_process.children(recursive=True)
            all_processes = [main_process] + children

            print(f'Terminating process {process.pid} with {len(children)} children', flush=True)

            # Step 1: Try graceful termination with SIGINT
            _send_signal_to_processes(all_processes, signal.SIGINT)

            # Wait for graceful termination
            if _wait_for_termination(all_processes, timeout=5):
                print(f'Process {process.pid} terminated gracefully')
                # Clean up the main subprocess
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                return True

            # Step 2: Escalate to SIGTERM
            print(f'Escalating to SIGTERM for process {process.pid}', flush=True)
            remaining_processes = _get_running_processes(all_processes)
            _send_signal_to_processes(remaining_processes, signal.SIGTERM)

            if _wait_for_termination(remaining_processes, timeout=5):
                print(f'Process {process.pid} terminated with SIGTERM')
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                return True

            # Step 3: First Force kill with SIGKILL
            print(f'Force killing process {process.pid}', flush=True)
            remaining_processes = _get_running_processes(all_processes)
            _send_signal_to_processes(remaining_processes, signal.SIGKILL)

            if _wait_for_termination(remaining_processes, timeout=5):
                print(f'Process {process.pid} terminated with SIGKILL')
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                return True

            # Step 4: Final Force kill with SIGKILL
            print(f'Force killing process {process.pid} again', flush=True)
            remaining_processes = _get_running_processes(all_processes)
            _send_signal_to_processes(remaining_processes, signal.SIGKILL)

            # Final wait
            final_success = _wait_for_termination(remaining_processes, timeout=5)

            # Always try to clean up the main subprocess
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass

            if not final_success:
                remaining = _get_running_processes(all_processes)
                if remaining:
                    print(f'Warning: {len(remaining)} processes still running after force kill')
                    # Log details about remaining processes
                    for proc in remaining:
                        try:
                            print(f'  - PID {proc.pid}: {proc.name()} (status: {proc.status()})')
                        except Exception:
                            pass
                    return False

            print(f'Process {process.pid} successfully terminated', flush=True)
            return True
        except psutil.NoSuchProcess:
            print(f'Process {process.pid} no longer exists', flush=True)
            return True
        except Exception as e:
            print(f'Error terminating process {process.pid}: {e}', flush=True)
            return False


class GeometryUtils:
    """Utility functions for geometric operations."""

    @staticmethod
    def are_poses_equal(
        pose1: Union[PoseStamped, None], pose2: Union[PoseStamped, None], tol: float = 1e-6
    ) -> bool:
        """Compare the positions of two PoseStamped messages."""
        if pose1 is None or pose2 is None:
            return False

        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return (dx * dx + dy * dy + dz * dz) < tol * tol


class RosUtils:
    """Utility functions for ROS 2 specific operations."""

    @staticmethod
    def get_node_params(yaml_file, node_name):
        """
        Load parameters for a specific node from a YAML file.

        Ignore the namespace prefix in the YAML keys.
        """
        if not yaml_file or not os.path.exists(yaml_file):
            return {}

        with open(yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if not data:
            return {}

        params = {}
        # 1. Global / wildcard
        if '/**' in data:
            params.update(data['/**'].get('ros__parameters', {}))

        # 2. Match node name (any namespace)
        # Search for exactly node_name or something ending with /node_name
        for key, value in data.items():
            if key == node_name or key.endswith('/' + node_name):
                params.update(value.get('ros__parameters', {}))

        return params

    @staticmethod
    def parent_namespace_from_fqn(fqn: str) -> str:
        """
        Return the parent namespace of a ROS 2 node given its fully qualified name.

        Examples
        --------
          /robot/node        -> /robot
          robot/node         -> /robot
          /r1/sensors/lidar  -> /r1
          r1/sensors/lidar   -> /r1
          /node              -> /
          node               -> /

        """
        if not fqn or fqn == '/':
            return '/'

        # Normalize: ensure leading slash
        if not fqn.startswith('/'):
            fqn = '/' + fqn

        fqn = fqn.rstrip('/')
        parts = fqn.split('/')

        # parts[0] == ""
        # ['', node]
        if len(parts) <= 2:
            return '/'

        # Top-level namespace (parent of node namespace)
        return '/' + parts[1]

    @staticmethod
    def prefix_topic_with_namespace(topic: str, namespace: str) -> str:
        """
        Prefix a topic with a given namespace.

        Examples
        --------
          topic: /odom, namespace: /          -> /odom
          topic: odom, namespace: /robot      -> /robot/odom

        """
        # Normalize topic
        if not topic.startswith('/'):
            topic = '/' + topic

        # Normalize namespace
        if not namespace or namespace == '/':
            return topic

        if not namespace.startswith('/'):
            namespace = '/' + namespace
        namespace = namespace.rstrip('/')

        return namespace + topic

    @staticmethod
    def join_frame(frame_prefix: str, frame_name: str) -> str:
        """
        Join a frame prefix and frame name.

        Examples
        --------
          frame_prefix: robot, frame_name: cmd_vel   -> robot/cmd_vel
          frame_prefix: /, frame_name: cmd_vel       -> cmd_vel
          frame_prefix: robot, frame_name: /cmd_vel  -> robot/cmd_vel

        """
        result = RosUtils.prefix_topic_with_namespace(frame_name, frame_prefix)
        if result.startswith('/'):
            result = result[1:]

        return result
