#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# Written: August 2024
# License: This code is licensed under the MIT License.
#
# Program: ROS Bag File Processing GUI Script
# Purpose: UI to process ROS bag files, applies various transformations, and provides utilities for managing bag files.

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QFileDialog, QStackedWidget, QListView, QDialog, QHBoxLayout, QLabel, QComboBox, QLineEdit, QPushButton, QCheckBox, QScrollArea
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
    def GetTopicsFromBag(cls, path):
        """
        Extract all unique topics from the given ROS bag file or all bag files in a folder.

        Args:
            path (str): Path to the ROS bag file or a folder containing ROS bag files.

        Returns:
            list: List of unique topics.
        """
        topics = set()
        try:
            if os.path.isdir(path):  # Check if the path is a folder
                bag_files = [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.bag')]
                if not bag_files:
                    print(f"No ROS bag files found in the folder: {path}")
                    return []
                for bag_file in bag_files:
                    print(f"Reading topics from bag file: {bag_file}")
                    with rosbag.Bag(bag_file, 'r') as bag:
                        for topic, _, _ in bag.read_messages():
                            topics.add(topic)
            elif os.path.isfile(path):  # Single bag file
                print(f"Reading topics from single bag file: {path}")
                with rosbag.Bag(path, 'r') as bag:
                    for topic, _, _ in bag.read_messages():
                        topics.add(topic)
            else:
                print(f"Invalid path: {path}")
        except Exception as e:
            print(f"Error reading bag file(s) at {path}: {e}")
        return sorted(topics)  # Return sorted list for better readability
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
        layout = QVBoxLayout()

        if msg:
            label = QLabel(msg)
            layout.addWidget(label)

        checkboxs = []
        for select in selectList:
            checkbox = QCheckBox(select)
            layout.addWidget(checkbox)
            checkboxs.append(checkbox)

        btn = QPushButton("OK")
        layout.addWidget(btn)

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
    def PromptForParameters(cls, FUNCTION_CHOICES):
        print("PromptForParameters called")
        app = QApplication(sys.argv)
        win = QWidget()

        # Main layout
        main_layout = QVBoxLayout()

        # Static widgets
        function_label = QLabel("Select Function:")
        function_combo = QComboBox()
        function_combo.addItems(FUNCTION_CHOICES)

        path_label = QLabel("Select Input Path (Folder or Bag File):")
        path_btn = QPushButton("Browse...")
        path_line_edit = QLineEdit()

        output_path_label = QLabel("Select Output Path (Folder or Bag File):")
        output_path_btn = QPushButton("Browse...")
        output_path_line_edit = QLineEdit()

        # Add static widgets to layout
        main_layout.addWidget(function_label)
        main_layout.addWidget(function_combo)
        main_layout.addWidget(path_label)
        main_layout.addWidget(path_btn)
        main_layout.addWidget(path_line_edit)
        main_layout.addWidget(output_path_label)
        main_layout.addWidget(output_path_btn)
        main_layout.addWidget(output_path_line_edit)

        # Scrollable area for dynamic widgets
        dynamic_widget_area = QWidget()
        dynamic_layout = QVBoxLayout()
        dynamic_widget_area.setLayout(dynamic_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(dynamic_widget_area)
        main_layout.addWidget(scroll_area)

        # OK and Back buttons at the bottom
        button_layout = QHBoxLayout()
        ok_btn = QPushButton("OK")
        back_btn = QPushButton("Back")
        button_layout.addWidget(back_btn)
        button_layout.addWidget(ok_btn)
        main_layout.addLayout(button_layout)

        # Dynamic widgets and params
        topic_checkboxes = []
        frame_id_input = QLineEdit()
        num_images_input = QLineEdit()
        seed_input = QLineEdit()
        params = {}

        def clear_dynamic_widgets():
            while dynamic_layout.count() > 0:
                child = dynamic_layout.takeAt(0)
                if child.widget():
                    child.widget().deleteLater()
        def clear_topic_widgets():
            """Clear only topic-related widgets from the dynamic layout."""
            for i in reversed(range(dynamic_layout.count())):
                widget = dynamic_layout.itemAt(i).widget()
                if isinstance(widget, QCheckBox) or (isinstance(widget, QLabel) and "Topic" in widget.text()):
                    dynamic_layout.takeAt(i)
                    widget.deleteLater()
        def get_active_topic_checkboxes():
            """
            Retrieve all active QCheckBox widgets for topics from the dynamic layout.

            Returns:
                list: List of active QCheckBox widgets.
            """
            return [dynamic_layout.itemAt(i).widget() for i in range(dynamic_layout.count())
                    if isinstance(dynamic_layout.itemAt(i).widget(), QCheckBox)]

        def update_dynamic_widgets():
            clear_topic_widgets()
            
            selected_function = function_combo.currentText()
            if selected_function == 'remove_topic':
                dynamic_layout.addWidget(QLabel("Select Topic(s):"))
                for topic in cls.GetTopicsFromBag(path_line_edit.text()):
                    checkbox = QCheckBox(topic)
                    dynamic_layout.addWidget(checkbox)
                    topic_checkboxes.append(checkbox)
            elif selected_function == 'change_frame_id':
                dynamic_layout.addWidget(QLabel("New Frame ID:"))
                dynamic_layout.addWidget(frame_id_input)
            elif selected_function == 'save_random_images':
                dynamic_layout.addWidget(QLabel("Select Topic(s):"))
                for topic in cls.GetTopicsFromBag(path_line_edit.text()):
                    checkbox = QCheckBox(topic)
                    dynamic_layout.addWidget(checkbox)
                    topic_checkboxes.append(checkbox)
                dynamic_layout.addWidget(QLabel("Number of Images:"))
                dynamic_layout.addWidget(num_images_input)
                dynamic_layout.addWidget(QLabel("Random Seed:"))
                dynamic_layout.addWidget(seed_input)


        def on_ok_button_click():
            params['function'] = function_combo.currentText()
            params['input_path'] = path_line_edit.text()
            params['output_path'] = output_path_line_edit.text()
            # Collect dynamic parameters
            if params['function'] == 'remove_topic':
                params['topics'] = [cb.text() for cb in get_active_topic_checkboxes() if cb.isChecked()]
            elif params['function'] == 'change_frame_id':
                params['new_frame_id'] = frame_id_input.text()
            elif params['function'] == 'save_random_images':
                params['num_images'] = int(num_images_input.text()) if num_images_input.text() else 10
                params['seed'] = int(seed_input.text()) if seed_input.text() else 42
                params['topics'] = [cb.text() for cb in get_active_topic_checkboxes() if cb.isChecked()]

            print(f"Parameters collected: {params}")
            app.exit()
        def on_path_button_click():
            selected_path = cls.GetPath()
            if selected_path and (os.path.isfile(selected_path) or os.path.isdir(selected_path)):
                path_line_edit.setText(selected_path)
                print(f"Selected bag file/folder path: {selected_path}")  # Debug output
                update_dynamic_widgets()  # Call only after a valid path is set
            else:
                print("Invalid path selected.")
        function_combo.currentIndexChanged.connect(update_dynamic_widgets)
        path_btn.clicked.connect(on_path_button_click)
        output_path_btn.clicked.connect(lambda: output_path_line_edit.setText(cls.GetPath()))
        ok_btn.clicked.connect(on_ok_button_click)
        back_btn.clicked.connect(clear_dynamic_widgets)

        win.setLayout(main_layout)
        win.setWindowTitle("Select Parameters")
        win.show()
        app.exec_()
        return params
