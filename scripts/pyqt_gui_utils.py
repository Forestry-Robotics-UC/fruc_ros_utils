#!/usr/bin/env python
# -*- coding:utf-8 -*-

from PyQt5.QtWidgets import QApplication, QFileDialog, QWidget, QVBoxLayout, QCheckBox, QPushButton, QLabel, QGridLayout
import sys
import os
import rosbag

class SimplePyQtGUIKit:
    @classmethod
    def GetPath(cls, caption="Select Folder or File", filefilter="Bag Files (*.bag);;All Files (*)"):
        """
        Open a file dialog to select files or folders.

        Args:
            caption (str): The dialog caption.
            filefilter (str): The file filter for the dialog.

        Returns:
            str: The selected path.
        """
        print("Starting GetPath")
        app = QApplication(sys.argv)
        
        try:
            # Try to get a directory first
            options = QFileDialog.Options()
            options |= QFileDialog.DontUseNativeDialog
            path = QFileDialog.getExistingDirectory(None, caption, options=options)

            # If no directory selected, try to get a file
            if not path:
                path, _ = QFileDialog.getOpenFileName(None, caption, "", filefilter, options=options)
            
            print(f"Selected path: {path}")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            app.quit()
        
        return path

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
    def PromptForParameters(cls, function):
        """
        Prompt the user for the necessary parameters based on the selected function.

        Args:
            function (str): The function to run.

        Returns:
            dict: Dictionary of parameters.
        """
        print(f"PromptForParameters called for function: {function}")
        input_path = cls.GetPath("Select Folder or Bag File")
        params = {'input_path': input_path}

        if os.path.isdir(input_path):
            bag_files = [os.path.join(input_path, f) for f in os.listdir(input_path) if f.endswith('.bag')]
            if not bag_files:
                print("No bag files found in the selected folder.")
                return None
            bag_path = bag_files[0]
        else:
            bag_path = input_path

        topics = cls.GetTopicsFromBag(bag_path)

        if function == 'remove_topic':
            selected_topics = [key for key, value in cls.GetCheckButtonSelect(topics, "Select Topics to Remove").items() if value]
            params['topics'] = selected_topics
            if not os.path.isdir(input_path):
                params['bagout'] = cls.GetPath("Select Output Bag File", "Bag Files (*.bag);;All Files (*)")
        elif function == 'change_frame_id':
            selected_topics = [key for key, value in cls.GetCheckButtonSelect(topics, "Select Topic to Change Frame ID").items() if value]
            new_frame_id = [key for key, value in cls.GetCheckButtonSelect(["new_frame_id1", "new_frame_id2"], "Select New Frame ID").items() if value][0]
            params['topics'] = selected_topics
            params['new_frame_id'] = new_frame_id
            if not os.path.isdir(input_path):
                params['bagout'] = cls.GetPath("Select Output Bag File", "Bag Files (*.bag);;All Files (*)")
        elif function == 'print_topic_sizes':
            pass

        print(f"Parameters collected: {params}")
        return params

# if __name__ == '__main__':
#     try:
#         params = SimplePyQtGUIKit.PromptForParameters('remove_topic')
#         print(params)
#     except Exception as e:
#         print(f"Error: {e}")
