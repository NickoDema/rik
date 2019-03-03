#!/usr/bin/env python

##   parsers.py
##   Created on: 04.03.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##
## TODO Full description

## python 2.7

import xml.dom.minidom as md

def urdf_to_model(urdf):

    #   RIK robot model is a dictionary which repeats taken urdf model
    model = {'links':  {},
             'joints': {} }

    md_model = md.parseString(urdf).getElementsByTagName('robot')[0]

    for child in robot.childNodes:

        if child.localName == 'joint':
            jname = child.getAttribute('name')
            jtype = child.getAttribute('type')
            if jtype == 'fixed' or jtype == 'floating':
                continue
            name = child.getAttribute('name')
            self.joint_list.append(name)
            if jtype == 'continuous':
                minval = -pi
                maxval = pi
            else:
                try:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))
                except:
                    rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                    continue

            safety_tags = child.getElementsByTagName('safety_controller')
            if self.use_small and len(safety_tags) == 1:
                tag = safety_tags[0]
                if tag.hasAttribute('soft_lower_limit'):
                    minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                if tag.hasAttribute('soft_upper_limit'):
                    maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

            mimic_tags = child.getElementsByTagName('mimic')
            if self.use_mimic and len(mimic_tags) == 1:
                tag = mimic_tags[0]
                entry = {'parent': tag.getAttribute('joint')}
                if tag.hasAttribute('multiplier'):
                    entry['factor'] = float(tag.getAttribute('multiplier'))
                if tag.hasAttribute('offset'):
                    entry['offset'] = float(tag.getAttribute('offset'))

                self.dependent_joints[name] = entry
                continue

            if name in self.dependent_joints:
                continue

            if self.zeros and name in self.zeros:
                zeroval = self.zeros[name]
            elif minval > 0 or maxval < 0:
                zeroval = (maxval + minval)/2
            else:
                zeroval = 0

            joint = {'min': minval, 'max': maxval, 'zero': zeroval}
            if self.pub_def_positions:
                joint['position'] = zeroval
            if self.pub_def_vels:
                joint['velocity'] = 0.0
            if self.pub_def_efforts:
                joint['effort'] = 0.0

            if jtype == 'continuous':
                joint['continuous'] = True
            self.free_joints[name] = joint


        if child.localName == 'link':
            pass

        if child.nodeType is child.TEXT_NODE:
            continue
