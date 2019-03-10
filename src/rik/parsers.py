#!/usr/bin/env python

##   parsers.py
##   Created on: 04.03.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##
## TODO Full description

## python 2.7

import xml.dom.minidom as md

def str_to_float(str):
     return [float(x) for x in str.split()]

def str_to_int(str):
     return [int(x) for x in str.split()]

def urdf_to_model(urdf):

    #   RIK robot model is a dictionary which repeats taken urdf model
    model = {'links':  {},
              'joints': {} }

    md_model = md.parseString(urdf).getElementsByTagName('robot')[0]

    for child in md_model.childNodes:

        if child.localName == 'joint':

            jname = child.getAttribute('name')
            jtype = child.getAttribute('type')

            md_jparent = child.getElementsByTagName('parent')[0]
            jparent = md_jparent.getAttribute('link')

            md_jchild = child.getElementsByTagName('child')[0]
            jchild = md_jchild.getAttribute('link')

            md_jorigin = child.getElementsByTagName('origin')[0]
            x, y, z = str_to_float(md_jorigin.getAttribute('xyz'))
            roll, pitch, yaw = str_to_float(md_jorigin.getAttribute('rpy'))
            jorigin = {'x': x,       'y': y,         'z': z,
                       'roll': roll, 'pitch': pitch, 'yaw': yaw}

            # correspond to fixed type
            jdata = {'type':   jtype,
                     'child':  jchild,
                     'parent': jparent,
                     'origin': jorigin}

            if jtype == 'revolute' or jtype == 'continuous':

                md_jaxis = child.getElementsByTagName('axis')[0]
                x, y, z = str_to_float(md_jaxis.getAttribute('xyz'))
                axis = {'x': x, 'y': y, 'z': z}

                jdata.update({'axis': axis})

            if jtype != 'revolute' and jtype != 'fixed':
                limit = {}

                # check for limit element in joint description
                try:
                    jlimit = child.getElementsByTagName('limit')[0]

                    if jlimit.hasAttribute('lower'):
                        jmin = float(jlimit.getAttribute('lower'))
                        limit.update({'lower': jmin})

                    if jlimit.hasAttribute('upper'):
                        jmax = float(jlimit.getAttribute('upper'))
                        limit.update({'upper': jmax})

                    if jlimit.hasAttribute('effort'):
                        jeffort = float(jlimit.getAttribute('effort'))
                        limit.update({'effort': jeffort})

                    if jlimit.hasAttribute('velocity'):
                        jvelocity = float(jlimit.getAttribute('velocity'))
                        limit.update({'velocity': jvelocity})
                except:
                    print(jname, " is not fixed, nor continuous, but limits are not specified!")

                jdata.update({'limit': limit})


            joint = {jname: jdata }
            model['joints'].update(joint)




            # safety_tags = child.getElementsByTagName('safety_controller')
            # if self.use_small and len(safety_tags) == 1:
            #     tag = safety_tags[0]
            #     if tag.hasAttribute('soft_lower_limit'):
            #         minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
            #     if tag.hasAttribute('soft_upper_limit'):
            #         maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))
            #
            # mimic_tags = child.getElementsByTagName('mimic')
            # if self.use_mimic and len(mimic_tags) == 1:
            #     tag = mimic_tags[0]
            #     entry = {'parent': tag.getAttribute('joint')}
            #     if tag.hasAttribute('multiplier'):
            #         entry['factor'] = float(tag.getAttribute('multiplier'))
            #     if tag.hasAttribute('offset'):
            #         entry['offset'] = float(tag.getAttribute('offset'))
            #
            #     self.dependent_joints[name] = entry
            #     continue
            #
            # if name in self.dependent_joints:
            #     continue
            #
            # if self.zeros and name in self.zeros:
            #     zeroval = self.zeros[name]
            # elif minval > 0 or maxval < 0:
            #     zeroval = (maxval + minval)/2
            # else:
            #     zeroval = 0
            #
            # joint = {'min': minval, 'max': maxval, 'zero': zeroval}
            # if self.pub_def_positions:
            #     joint['position'] = zeroval
            # if self.pub_def_vels:
            #     joint['velocity'] = 0.0
            # if self.pub_def_efforts:
            #     joint['effort'] = 0.0
            #
            # if jtype == 'continuous':
            #     joint['continuous'] = True
            # self.free_joints[name] = joint


        if child.localName == 'link':
            lname = child.getAttribute('name')
            model['links'].update({lname: {}})

        if child.nodeType is child.TEXT_NODE:
            continue

    return model
