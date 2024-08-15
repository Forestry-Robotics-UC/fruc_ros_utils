#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# Written: August 2024
# License: This code is licensed under the MIT License.
#
# Program: ROS Bag File Processing GUI Script
# Purpose: UI to processes ROS bag files, applies various transformations, and provides utilities for managing bag files.

from PyQt5.QtWidgets import QApplication, QFileDialog, QWidget, QVBoxLayout, QCheckBox, QPushButton, QLabel, QGridLayout, QComboBox, QLineEdit, QStackedWidget, QListView, QDialog
import sys
import os
import rosbag

class SimplePyQtGUIKit:
    @classmethod
    def GetPath(cls, caption="Select Folder or File", directory='', filter="Bag Files (*.bag);;All Files (*)", initialFilter='', options=None):
        """
        Open a file dialog to select files or folders.

        Args:
            caption (str): The dialog caption.
            directory (str): The initial directory to start in.
            filter (str): The file filter for the dialog.
            initialFilter (str): The initial filter to select.
            options (QFileDialog.Options): Options for the file dialog.

        Returns:
            str: The selected path or concatenated paths.
        """
        app = QApplication.instance()
        if app is None:
            app = QApplication(sys.argv)

        def updateText():
            # Update the contents of the line edit widget with the selected files
            selected = []
            for index in view.selectionModel().selectedRows():
                selected.append('"{}"'.format(index.data()))
            lineEdit.setText(' '.join(selected))

        dialog = QFileDialog(parent=None, windowTitle=caption)
        dialog.setFileMode(QFileDialog.ExistingFiles)
        if options:
            dialog.setOptions(options)
        dialog.setOption(QFileDialog.DontUseNativeDialog, True)
        if directory:
            dialog.setDirectory(directory)
        if filter:
            dialog.setNameFilter(filter)
            if initialFilter:
                dialog.selectNameFilter(initialFilter)

        # Override accept to allow selecting directories as well as files
        dialog.accept = lambda: QDialog.accept(dialog)

        # Access the stacked widget containing the views
        stackedWidget = dialog.findChild(QStackedWidget)
        view = stackedWidget.findChild(QListView)
        view.selectionModel().selectionChanged.connect(updateText)

        lineEdit = dialog.findChild(QLineEdit)
        # Clear the line edit contents whenever the current directory changes
        dialog.directoryEntered.connect(lambda: lineEdit.setText(''))

        dialog.exec_()
        selected_files = dialog.selectedFiles()

        # Return the first selected path or join multiple selections into a string
        return selected_files[0] if selected_files else ''

    @classmethod
    def GetTopicsFromBag(cls, bag_path):
        """
        Extract all unique topics from the given ROS bag file.

        Args:
            bag_path (str): Path to the ROS bag file.

        Returns:
            list: List of unique topics.
        """
        topics = set()
        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                for topic, _, _ in bag.read_messages():
                    topics.add(topic)
        except Exception as e:
            print(f"Error reading bag file: {e}")
        return list(topics)

    @classmethod
    def GetCheckButtonSelect(cls, selectList, title="Select", msg=""):
        """
        Get selected check button options.

        Args:
            selectList (list): List of options to select from.
            title (str): Window name.
            msg (str): Label of the check button.

        Returns:
            dict: Dictionary of selected options.
        """
        print("Starting GetCheckButtonSelect")
        app = QApplication(sys.argv)
        win = QWidget()
        layout = QGridLayout()

        if msg:
            label = QLabel(msg)
            layout.addWidget(label, 0, 0)

        checkboxs = []
        for i, select in enumerate(selectList):
            checkbox = QCheckBox(select)
            layout.addWidget(checkbox, i + 1, 0)
            checkboxs.append(checkbox)

        btn = QPushButton("OK")
        layout.addWidget(btn, len(selectList) + 1, 0)

        def on_button_click():
            app.quit()

        btn.clicked.connect(on_button_click)
        win.setLayout(layout)
        win.setWindowTitle(title)
        win.show()
        app.exec_()

        result = {select: checkbox.isChecked() for select, checkbox in zip(selectList, checkboxs)}
        print(f"Selected options: {result}")
        return result

    @classmethod
    def PromptForParameters(cls):
        """
        Prompt the user for the function to run and the necessary parameters.

        Returns:
            dict: Dictionary of parameters.
        """
        print("PromptForParameters called")
        app = QApplication(sys.argv)
        win = QWidget()
        layout = QVBoxLayout()

        # Function selection
        function_label = QLabel("Select Function:")
        layout.addWidget(function_label)
        function_combo = QComboBox()
        functions = ['remove_topic', 'change_frame_id', 'print_topic_sizes', 'convert_imu_to_enu', 'merge_bags']
        function_combo.addItems(functions)
        layout.addWidget(function_combo)

        # Input/Output path selection
        path_label = QLabel("Select Input Path (Folder or Bag File):")
        layout.addWidget(path_label)
        path_btn = QPushButton("Browse...")
        layout.addWidget(path_btn)
        path_line_edit = QLineEdit()
        layout.addWidget(path_line_edit)

        output_path_label = QLabel("Select Output Path (Folder or Bag File):")
        layout.addWidget(output_path_label)
        output_path_btn = QPushButton("Browse...")
        layout.addWidget(output_path_btn)
        output_path_line_edit = QLineEdit()
        layout.addWidget(output_path_line_edit)

        # Dynamic argument widgets
        topic_label = QLabel("Select Topic(s):")
        layout.addWidget(topic_label)
        topic_checkboxes = []
        frame_id_label = QLabel("New Frame ID:")
        frame_id_input = QLineEdit()

        # OK button
        ok_btn = QPushButton("OK")
        layout.addWidget(ok_btn)

        params = {}

        def update_dynamic_widgets():
            selected_function = function_combo.currentText()

            # Clear previous dynamic widgets
            for widget in topic_checkboxes:
                layout.removeWidget(widget)
                widget.setParent(None)
            layout.removeWidget(frame_id_label)
            layout.removeWidget(frame_id_input)
            frame_id_input.setParent(None)

            # Initialize topics list
            topics = []

            # Add widgets based on selected function
            input_path = path_line_edit.text()
            first_bag_file = None

            if os.path.isdir(input_path):
                # Handle folder selection
                bag_files = [f for f in os.listdir(input_path) if f.endswith('.bag')]
                if bag_files:
                    first_bag_file = os.path.join(input_path, bag_files[0])
                else:
                    print("No bag files found in the selected folder.")
                    return
            else:
                # Handle file selection
                first_bag_file = input_path

            if first_bag_file:
                try:
                    topics = cls.GetTopicsFromBag(first_bag_file)
                except Exception as e:
                    print(f"Error reading bag file: {e}")
                    topics = []

            if selected_function in ['remove_topic', 'change_frame_id', 'convert_imu_to_enu']:
                for i, topic in enumerate(topics):
                    checkbox = QCheckBox(topic)
                    layout.addWidget(checkbox)
                    topic_checkboxes.append(checkbox)

            if selected_function == 'change_frame_id':
                layout.addWidget(frame_id_label)
                layout.addWidget(frame_id_input)


        def on_path_button_click():
            selected_path = cls.GetPath()
            if selected_path:
                path_line_edit.setText(selected_path)
                update_dynamic_widgets()

        def on_output_path_button_click():
            selected_path = cls.GetPath()
            if selected_path:
                output_path_line_edit.setText(selected_path)

        def on_ok_button_click():
            selected_function = function_combo.currentText()
            params.update({
                'function': selected_function,
                'input_path': path_line_edit.text(),
                'output_path': output_path_line_edit.text(),
                'topics': [cb.text() for cb in topic_checkboxes if cb.isChecked()] if topic_checkboxes else None
            })

            if selected_function == 'change_frame_id':
                params['new_frame_id'] = frame_id_input.text()

            print(f"Parameters collected: {params}")
            app.quit()

        function_combo.currentIndexChanged.connect(update_dynamic_widgets)
        path_btn.clicked.connect(on_path_button_click)
        output_path_btn.clicked.connect(on_output_path_button_click)
        ok_btn.clicked.connect(on_ok_button_click)

        win.setLayout(layout)
        win.setWindowTitle("Select Parameters")
        win.show()
        app.exec_()

        return params
