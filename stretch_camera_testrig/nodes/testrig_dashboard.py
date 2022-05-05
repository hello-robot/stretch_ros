#!/usr/bin/env python
from Tkinter import *
import ttk
from testrig_analyze_data import TestRig_Analyze
import os
import yaml
from subprocess import Popen, PIPE
import numpy as np
from tabulate import tabulate
import math
import pprint
import copy


class TestRig_dashboard():
    def __init__(self):
        self.ros_package_dir = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_camera_testrig')
        self.data_directory = self.ros_package_dir + '/data'
        self.nominal_poses_filename = self.ros_package_dir + '/config/testrig_marker_info.yaml'

        self.data_keys = ['base_left_marker_pose',
                          'base_right_marker_pose',
                          'wrist_inside_marker_pose',
                          'wrist_top_marker_pose',
                          'shoulder_marker_pose']
        self.metrics_keys = ["maximum", "mean", "median", "rmse"]
        self.error_keys = ["angle_rotation", "euclidean_error"]
        self.window = Tk()
        self.window.title("Camera Test rig Dashboard")
        self.window.geometry('800x900')

        self.data_file_name = None
        self.test_rig = None
        self.table_entry = None
        self.result_file_name = None

        #################### Top Pane ##########################

        self.frame_top = Frame(self.window, width=100, height=25)
        self.frame_top.grid(column=0, row=0, sticky="w")

        self.load_data_recent_btn = Button(self.frame_top, text="Load Most Recent Data",
                                           command=self.load_recent_clicked, font=("Arial", 14))
        self.load_data_recent_btn.pack(side=TOP)

        self.data_file_entry_lbl = Label(self.frame_top, text="Enter Data file path ")
        self.data_file_entry_lbl.pack(side=LEFT)

        self.data_file_path = StringVar()
        self.data_file_path_entry = Entry(self.frame_top, textvariable=self.data_file_path, width=30)
        self.data_file_path_entry.pack(side=LEFT)

        self.load_data_btn = Button(self.frame_top, text="Load", command=self.load_clicked)
        self.load_data_btn.pack(side=LEFT)

        self.test_capture_id_lbl = Label(self.window, text=" ")
        self.test_capture_id_lbl.grid(column=0, row=1, sticky="w")

        self.LOGGER = Label(self.window, text="", anchor='w')
        self.LOGGER.grid(column=0, row=2, sticky="w")

        ############## Nominal Poses Update Section Pane ###########

        x_off = 0
        y_off = 0

        self.nominal_poses_label = Label(self.window, text="Nominal Poses", font=("Arial", 18))
        self.nominal_poses_label.place(x=x_off + 50, y=y_off + 120)

        # Choose Setup Nominal Poses File (WIP)
        # self.marker_info_config_filename = StringVar()
        # self.choose_marker_info_config = ttk.Combobox(self.window, width=30, textvariable=self.marker_info_config_filename)
        # self.choose_marker_info_config['values'] = os.listdir(self.ros_package_dir+'/config')
        # self.choose_marker_info_config.place(x=x_off + 250, y=y_off + 123)

        self.matrix_text_var = None
        self.matrix_entries = None
        self.create_matrix_entry(x_off + 30, y_off + 150)
        self.update_nominal_poses_btn = Button(self.window, text="Update Nominal Poses",
                                               command=self.update_nominal_poses)
        self.update_nominal_poses_btn.place(x=x_off + 20, y=y_off + 280)

        self.nominal_poses_radiobuttons = []
        self.radio_var = IntVar()
        vi = 0
        for key in self.data_keys:
            r = Radiobutton(self.window, text=key, value=vi, var=self.radio_var,
                            command=self.radiobutton_sel)
            vi = vi + 1
            self.nominal_poses_radiobuttons.append(r)
        for i in range(len(self.nominal_poses_radiobuttons)):
            self.nominal_poses_radiobuttons[i].place(x=x_off + 250, y=y_off + 150 + i * 20)

        ################## Test Results Display Pane ########################
        self.x_off_mid = 0
        self.y_off_mid = 0
        self.test_results_title = Label(self.window, text="Testrig Computed Results", font=("Arial", 18))
        self.test_results_title.place(x=self.x_off_mid + 50, y=self.y_off_mid + 330)

        self.result_file_entry_lbl = Label(self.window, text="Result file path ")
        self.result_file_entry_lbl.place(x=self.x_off_mid + 10, y=self.y_off_mid + 365)

        self.result_file_name = StringVar()
        self.result_file_name_entry = Entry(self.window, textvariable=self.result_file_name, width=30)
        self.result_file_name_entry.place(x=self.x_off_mid + 120, y=self.y_off_mid + 365)

        self.load_result_btn = Button(self.window, text="Show Result", command=self.result_load_clicked)
        self.load_result_btn.place(x=self.x_off_mid + 370, y=self.y_off_mid + 365)

        self.metric_title = None
        self.test_info_title = None

        ################### Bottom Pane #########################
        x_off_bottom = 0
        y_off_bottom = 300
        self.save_results_btn = Button(self.window, text="Save Computed Results", command=self.save_results_clicked,
                                       font=("Arial", 14))
        self.save_results_btn.place(x=x_off_bottom + 150, y=y_off_bottom + 500)

        self.run_new_test_btn = Button(self.window, text="Run a new Test", command=self.run_new_test, state='disabled',
                                       font=("Arial", 14))
        self.run_new_test_btn.place(x=x_off_bottom + 150, y=y_off_bottom + 540)

        self.target_samples = IntVar()
        self.target_samples_entry = Entry(self.window, textvariable=self.target_samples, width=6, state='disabled')
        self.target_samples_entry.place(x=x_off_bottom + 450, y=y_off_bottom + 546)

        self.target_samples_lbl = Label(self.window, text="Enter Target frames:")
        self.target_samples_lbl.place(x=x_off_bottom + 310, y=y_off_bottom + 546)

    def test_data_info(self, x_pos, y_pos, results_dict=None):
        self.test_info_title = Label(self.window, text='Test Info', font=("Arial", 13, 'bold'))
        self.test_info_title.place(x=x_pos, y=y_pos - 25)
        info_dict = copy.deepcopy(results_dict)
        if not results_dict:
            info_dict = copy.deepcopy(self.test_rig.test_results_dict)

        try:
            del info_dict['performance_metrics']
        except KeyError:
            print('Performance Metrics not found.')
        for key in self.data_keys:
            null_frames_percent = (float(info_dict['null_frames'][key]) / float(info_dict['number_samples'])) * 100
            info_dict['null_frames'][key] = "{} ({:.1f}%)".format(info_dict['null_frames'][key], null_frames_percent)
        info_txt = str(yaml.safe_dump(info_dict, allow_unicode=True, default_flow_style=False, indent=4))
        self.info_print = Label(self.window, text=info_txt, anchor="w", font=("Arial", 11), justify=LEFT)
        self.info_print.place(x=x_pos, y=y_pos)

    def metrics_table_print(self, pos_x, pos_y, error_key, results_dict=None):
        n_rows = len(self.data_keys) + 1
        n_columns = len(self.metrics_keys) + 1
        data_list = []

        if not results_dict:
            results_dict = self.test_rig.test_results_dict
        for key in self.data_keys:
            d = [key]
            for mkey in self.metrics_keys:
                results = results_dict['performance_metrics'][error_key]
                d.append(results[key][mkey])
            data_list.append(d)
        mkeys = self.metrics_keys
        mkeys = ['Marker Tags'] + mkeys
        data_list = [mkeys] + data_list
        x2 = 0
        y2 = 0

        self.metric_title = Label(self.window, text=error_key + ' Metrics', font=("Arial", 13, 'bold'))

        if error_key == 'angle_rotation':
            self.metric_title.configure(text=error_key + ' Metrics (deg)')
        if error_key == 'euclidean_error':
            self.metric_title.configure(text=error_key + ' Metrics (meter)')

        self.metric_title.place(x=pos_x, y=pos_y - 25)
        for i in range(n_rows):
            for j in range(n_columns):
                d = data_list[i][j]
                if type(d) == type(float()):
                    d = format(d, '.6f')
                if j == 0:
                    self.table_entry = Entry(self.window, width=22, font=('Arial', 10, 'bold'))
                    self.table_entry.place(x=pos_x + x2 - 110, y=pos_y + y2)
                    self.table_entry.insert(END, d)
                elif i == 0:
                    self.table_entry = Entry(self.window, width=8, font=('Arial', 10, 'bold'))
                    self.table_entry.place(x=pos_x + x2, y=pos_y + y2)
                    self.table_entry.insert(END, d)
                else:
                    self.table_entry = Entry(self.window, width=8)
                    self.table_entry.place(x=pos_x + x2, y=pos_y + y2)
                    self.table_entry.insert(END, d)
                x2 += 70
            y2 += 30
            x2 = 0

    def create_matrix_entry(self, pos_x, pos_y):
        self.matrix_text_var = []
        self.matrix_entries = []
        x2 = 0
        y2 = 0
        rows, cols = (4, 4)
        for i in range(rows):
            self.matrix_text_var.append([])
            self.matrix_entries.append([])
            for j in range(cols):
                # append your StringVar and Entry
                self.matrix_text_var[i].append(StringVar())
                self.matrix_entries[i].append(Entry(self.window, textvariable=self.matrix_text_var[i][j], width=6))
                self.matrix_entries[i][j].place(x=pos_x + x2, y=pos_y + y2)
                x2 += 55
            y2 += 30
            x2 = 0

    def run_new_test(self):
        ros_log = None
        try:
            ros_log = Popen("roslaunch stretch_camera_testrig testrig_collect_data.launch", shell=True, bufsize=64,
                            stdin=PIPE, stdout=PIPE,
                            close_fds=True).stdout.read()
        except:
            print('Unable to Collect Testrig Data.')
        print(ros_log)

    def radiobutton_sel(self):
        selected_data_key = self.data_keys[self.radio_var.get()]
        self.log('selected {}'.format(selected_data_key))
        nominal_poses_dict = self.get_nominal_pose_dict()
        for key in self.data_keys:
            if key == selected_data_key:
                print('Current Value:')
                print(nominal_poses_dict[key])
                for i in range(4):
                    for j in range(4):
                        self.matrix_text_var[i][j].set(nominal_poses_dict[key][i][j])
        # print(self.matrix_text_var)
        # print(self.matrix_entries)

    def get_nominal_pose_dict(self):
        filename = self.nominal_poses_filename
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
            nominal_poses_dict = {}
            for key in data['testrig_aruco_marker_info'].keys():
                nominal_poses_dict[key] = np.array(data['testrig_aruco_marker_info'][key])
            return nominal_poses_dict
        except IOError:
            print('[Error] Unable to open Testrig Nominal Poses file: {0}'.format(filename))

    def get_current_entry_matrix(self):
        mat = None
        try:
            mat = []
            rows, cols = (4, 4)
            for i in range(rows):
                mat.append([])
                for j in range(cols):
                    mat[i].append(float(self.matrix_entries[i][j].get()))
            return np.array(mat)
        except:
            self.log('Non-Valid Matrix Entered.')
            return None

    def set_nominal_pose_dict(self, pose_matrix, marker_name):
        filename = self.nominal_poses_filename
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
            data['testrig_aruco_marker_info'][marker_name] = pose_matrix.tolist()
            with open(filename, 'w') as file:
                yaml.dump(data, file)
        except IOError:
            print('[Error] Unable to update Nominal Poses file: {0}'.format(filename))
            self.log('Unable to set Nominal Pose Matrix')

    def update_nominal_poses(self):
        mat = self.get_current_entry_matrix()
        selected_data_key = self.data_keys[self.radio_var.get()]
        print(selected_data_key)
        self.set_nominal_pose_dict(mat, selected_data_key)
        self.log('Updated "{}" Nominal Pose.'.format(selected_data_key))

    def log(self, txt):
        self.LOGGER.configure(text=txt)
        print(txt)
        self.window.update_idletasks()

    def save_results_clicked(self):
        if self.test_rig:
            self.test_rig.save_error_computations()
            self.test_rig.save_testrig_results()
            self.log('Testrig Analyze results saved in /stretch_camera_testrig/data/results')

    def result_load_clicked(self):
        try:
            with open(self.data_directory + '/results/' + self.result_file_name.get(), 'r') as file:
                data = yaml.safe_load(file)
            self.display_results(data)
            self.test_capture_id_lbl.configure(text="Testrig Data Capture ID : {} loaded".format(data['capture_id']))
            self.log('Loaded Result File : {}'.format(self.result_file_name.get()))
        except IOError:
            self.log('Unable to load result file.')

    def load_clicked(self):
        if len(self.data_file_path.get()) > 0:
            try:
                self.log('Loading given Test data file....')
                self.test_rig = self.get_testrig(self.data_directory + '/' + self.data_file_path.get())
                self.test_capture_id_lbl.configure(
                    text="Testrig Data Capture ID : {} loaded".format(self.test_rig.data_capture_date))
                self.log('Loaded Test Rig data file : {}'.format(self.test_rig.data_filename.split('/')[-1]))
            except IOError:
                self.log('Unable to load given data file:{}'.format(self.data_file_path.get()))
        else:
            self.log('Enter a data file path.')

    def load_recent_clicked(self):
        self.log('Loading most Recent Test data file....')
        self.test_rig = self.get_testrig()
        self.test_capture_id_lbl.configure(
            text="Testrig Capture ID : {} loaded".format(self.test_rig.data_capture_date))
        self.log('Loaded Test Rig data file : {}'.format(self.test_rig.data_filename.split('/')[-1]))

    def display_results(self, results_dict=None):
        self.metrics_table_print(self.x_off_mid + 130, self.y_off_mid + 420, 'euclidean_error', results_dict)
        self.metrics_table_print(self.x_off_mid + 130, self.y_off_mid + 620, 'angle_rotation', results_dict)
        self.test_data_info(480, 140, results_dict)

    def get_testrig(self, data_file=None):
        test_rig = TestRig_Analyze(data_file)
        test_rig.generate_error_observations()
        test_rig.populate_performance_metrics()
        self.test_rig = test_rig
        self.display_results()
        self.result_file_name.set('')
        return test_rig

    def mainloop(self):
        self.window.mainloop()


if __name__ == "__main__":
    dashboard = TestRig_dashboard()
    dashboard.mainloop()
