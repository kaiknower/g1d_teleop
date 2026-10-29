"""Package entrypoint boundaries; every child and robot connection is inert."""
import builtins
from contextlib import ExitStack, redirect_stdout
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from g1d_vr_teleop import cli, config
from g1d_vr_teleop.control import launcher, manager
from g1d_vr_teleop.recording import manager as recording_manager


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


class CliTests(unittest.TestCase):
    def test_top_level_help_does_not_import_any_command_or_sdk(self):
        for argv in ([], ['--help']):
            with self.subTest(argv=argv), redirect_stdout(io.StringIO()) as output, \
                    patch.object(cli, 'import_module') as load:
                if argv:
                    with self.assertRaises(SystemExit) as ended:
                        cli.main(argv)
                    self.assertEqual(ended.exception.code, 0)
                else:
                    self.assertEqual(cli.main(argv), 0)
                load.assert_not_called()
                self.assertIn('g1d-vr', output.getvalue())

    def test_control_and_input_help_exit_before_hardware_imports(self):
        original_import = builtins.__import__
        hardware_roots = {'ial_g1d', 'unitree_sdk2py', 'cyclonedds', 'televuer', 'pinocchio'}

        def guarded_import(name, *args, **kwargs):
            if name.split('.')[0] in hardware_roots:
                raise AssertionError('Help imported hardware: ' + name)
            return original_import(name, *args, **kwargs)

        for command in ('input', 'run', 'session', 'bind', 'restore', 'recover', 'record-actions'):
            with self.subTest(command=command), \
                    patch('builtins.__import__', side_effect=guarded_import), \
                    patch.object(launcher.subprocess, 'Popen') as spawn, \
                    patch.object(Path, 'mkdir') as mkdir, \
                    redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit) as ended:
                    cli.main([command, '--help'])
                self.assertEqual(ended.exception.code, 0)
                spawn.assert_not_called()
                mkdir.assert_not_called()

    def test_dispatch_loads_only_selected_command_and_preserves_arguments(self):
        cases = {
            'input': 'input.server', 'run': 'control.manager',
            'session': 'control.launcher', 'bind': 'control.bind',
            'restore': 'control.restore', 'recover': 'control.recover',
            'record-video': 'recording.video', 'record-actions': 'recording.actions',
        }
        arguments = ['--directory', 'a path with spaces', '--execute', '--', '-literal']
        for command, module_name in cases.items():
            handler = Mock(return_value=37)
            module = SimpleNamespace(main=handler)
            with self.subTest(command=command), \
                    patch.object(cli, 'import_module', return_value=module) as load:
                self.assertEqual(cli.main([command, *arguments]), 37)
                load.assert_called_once_with('g1d_vr_teleop.' + module_name)
                handler.assert_called_once_with(arguments)

    def test_run_without_execute_never_connects_spawns_or_creates_state(self):
        original_import = builtins.__import__
        hardware_roots = {'ial_g1d', 'unitree_sdk2py', 'cyclonedds', 'televuer', 'pinocchio'}

        def guarded_import(name, *args, **kwargs):
            if name.split('.')[0] in hardware_roots:
                raise AssertionError('Read-only run imported hardware: ' + name)
            return original_import(name, *args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            state = Path(directory) / 'not-created'
            with patch.object(manager, 'BASE', state), \
                    patch.object(manager, 'SessionManager') as controller, \
                    patch.object(manager, 'require_ready') as ready, \
                    patch.object(manager, 'launch_session') as launch, \
                    patch.object(launcher.subprocess, 'Popen') as spawn, \
                    patch('builtins.__import__', side_effect=guarded_import), \
                    redirect_stdout(io.StringIO()) as output:
                self.assertEqual(cli.main(['run']), 0)
            controller.assert_not_called()
            ready.assert_not_called()
            launch.assert_not_called()
            spawn.assert_not_called()
            self.assertFalse(state.exists())
            report = json.loads(output.getvalue())
            self.assertTrue(report['execute_required'])
            self.assertEqual(report['motion_commands_sent'], 0)


class ManagerIdentityTests(unittest.TestCase):
    def test_supported_python_and_console_manager_entrypoints(self):
        commands = (
            ['python', '-m', 'g1d_vr_teleop.control.manager', '--execute'],
            ['/env/bin/python', '-u', '-m', 'g1d_vr_teleop', 'run', '--execute'],
            ['/env/bin/g1d-vr', 'run', '--execute'],
            ['/env/bin/python', '/env/bin/g1d-vr', 'run', '--execute'],
        )
        for argv in commands:
            with self.subTest(argv=argv):
                self.assertTrue(launcher.is_manager_command(argv))

    def test_other_commands_and_manager_names_inside_script_arguments_are_rejected(self):
        commands = (
            [], ['python', '-m'], ['g1d-vr'],
            ['python', '-m', 'g1d_vr_teleop', 'session', '--execute'],
            ['g1d-vr', 'restore', '--execute'],
            ['python', '-m', 'unrelated', 'run'],
            ['python', '-m', 'g1d_vr_teleop.control.manager_extra'],
            ['python', 'other.py', '-m', 'g1d_vr_teleop', 'run'],
            ['python', '-c', 'pass', '-m', 'g1d_vr_teleop', 'run'],
            ['python', 'other.py', '/env/bin/g1d-vr', 'run'],
        )
        for argv in commands:
            with self.subTest(argv=argv):
                self.assertFalse(launcher.is_manager_command(argv))


class InstalledEnvironmentTests(unittest.TestCase):
    def test_wheel_does_not_inject_another_interpreters_site_packages(self):
        with tempfile.TemporaryDirectory() as directory:
            foreign_packages=Path(directory)/'control-env'/'site-packages'
            with patch.object(config,'SOURCE_ROOT',foreign_packages), \
                    patch.dict(os.environ,{'PYTHONPATH':''}):
                env=config.child_environment()
            self.assertNotIn(str(foreign_packages),env['PYTHONPATH'].split(os.pathsep))


class ChildEntrypointTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name)
        self.state = self.workspace / 'state'
        self.records = self.workspace / 'records'
        self.xr = self.workspace / 'dependencies' / 'xr'
        self.client = self.workspace / 'dependencies' / 'client'
        self.extra_import_path = self.workspace / 'existing-imports'
        self.patches = ExitStack()
        self.addCleanup(self.patches.close)
        for name, value in (
            ('ROOT', self.workspace), ('STATE_ROOT', self.state),
            ('RECORD_ROOT', self.records), ('XR_ROOT', self.xr),
            ('CLIENT_SOURCE', self.client),
        ):
            self.patches.enter_context(patch.object(config, name, value))
        self.patches.enter_context(patch.dict(os.environ, {
            'PYTHONPATH': str(self.extra_import_path),
            'G1D_VR_WORKSPACE': 'relative-parent-setting',
            'G1D_VR_STATE_ROOT': 'state',
        }))

    def assert_child_environment(self, env):
        self.assertEqual(env['G1D_VR_WORKSPACE'], str(self.workspace))
        self.assertEqual(env['G1D_VR_STATE_ROOT'], str(self.state))
        self.assertEqual(env['G1D_VR_RECORD_ROOT'], str(self.records))
        paths = env['PYTHONPATH'].split(os.pathsep)
        self.assertEqual(paths[0], str(config.SOURCE_ROOT))
        self.assertTrue((Path(paths[0]) / 'g1d_vr_teleop' / '__init__.py').is_file())
        self.assertIn(str(self.client), paths)
        self.assertIn(str(self.xr), paths)
        self.assertIn(str(self.extra_import_path), paths)
        self.assertNotEqual(paths[0], str(self.state))

    def test_launch_uses_one_state_root_and_importable_module_after_cwd_change(self):
        save(self.state / 'initial-pose.json', {'boot_id': 'test-boot'})
        child = SimpleNamespace(pid=314159)
        python = str(self.workspace / 'control-env' / 'bin' / 'python')
        with patch.object(launcher, 'BASE', self.state), \
                patch.object(launcher, 'RECORD_ROOT', self.records), \
                patch.object(launcher, 'XR_ROOT', self.xr), \
                patch.object(launcher, 'PYTHON', python), \
                patch.object(launcher, 'require_ready') as ready, \
                patch.object(launcher, 'current_boot_id', return_value='test-boot'), \
                patch.object(launcher.subprocess, 'Popen', return_value=child) as spawn:
            meta, result = launcher.launch_session(execute=True)
        ready.assert_called_once_with()
        spawn.assert_called_once()
        self.assertIs(result, child)
        argv = spawn.call_args.args[0]
        self.assertEqual(argv[:3], [python, '-m', 'g1d_vr_teleop.control.session'])
        control_directory = Path(argv[argv.index('--directory') + 1])
        record_directory = Path(argv[argv.index('--record-output') + 1])
        self.assertEqual(control_directory.parent, self.state)
        self.assertEqual(record_directory.parent, self.records)
        self.assertEqual(spawn.call_args.kwargs['cwd'], str(self.xr / 'teleop'))
        self.assertTrue(spawn.call_args.kwargs['start_new_session'])
        self.assert_child_environment(spawn.call_args.kwargs['env'])
        self.assertEqual(json.loads((self.state / 'current.json').read_text()), meta)
        self.assertEqual(meta['fixed_initial_pose'], str(self.state / 'initial-pose.json'))
        self.assertEqual(meta['control_directory'], str(control_directory))
        self.assertTrue((self.state / 'launch-owner.lock').is_file())
        self.assertTrue((self.state / 'manager-owner.lock').is_file())
        self.assertFalse((self.workspace / 'current.json').exists())
        status = json.loads((control_directory / 'control-status.json').read_text())
        self.assertFalse(status['robot_control_enabled'])
        self.assertEqual(status['commands_sent'], 0)

    def test_recorders_use_package_modules_with_their_own_interpreters(self):
        recording = recording_manager.Recording(self.records / 'episode')
        camera_python = str(self.workspace / 'camera-env' / 'bin' / 'python')
        control_python = str(self.workspace / 'control-env' / 'bin' / 'python')
        video = {'status': 'recording', 'receive_stats': {
            name: {'received_frames': 1}
            for name in ('head_stereo', 'left_wrist', 'right_wrist')
        }}
        actions = {'status': 'recording', 'received': {
            topic: 1 for topic in ('rt/lowstate', 'rt/dex1/left/state', 'rt/dex1/right/state')
        }}

        def fake_spawn(argv, **kwargs):
            output = Path(argv[argv.index('--output') + 1])
            report = video if output.name == 'video' else actions
            save(output / 'recording.json', report)
            return SimpleNamespace(poll=lambda: None)

        try:
            with patch.object(recording_manager, 'CAMERA', camera_python), \
                    patch.object(recording_manager, 'TV', control_python), \
                    patch.object(recording_manager.subprocess, 'Popen', side_effect=fake_spawn) as spawn:
                recording.prepare(threading.Event())
            self.assertEqual(spawn.call_count, 2)
            expected = (
                (camera_python, 'g1d_vr_teleop.recording.video', 'video'),
                (control_python, 'g1d_vr_teleop.recording.actions', 'action-state'),
            )
            for call, (python, module, name) in zip(spawn.call_args_list, expected):
                argv = call.args[0]
                self.assertEqual(argv[:3], [python, '-m', module])
                self.assertEqual(Path(argv[argv.index('--output') + 1]), recording.raw / name)
                self.assertEqual(Path(argv[argv.index('--stop-file') + 1]), recording.raw / (name + '.stop'))
                self.assert_child_environment(call.kwargs['env'])
            self.assertIsNone(recording.interval)
            self.assertFalse((recording.output / 'task').exists())
        finally:
            for log in recording.logs:
                log.close()


if __name__ == '__main__':
    unittest.main()
