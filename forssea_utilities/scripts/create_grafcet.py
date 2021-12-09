#!/usr/bin/env python

import yaml
import sys
import getopt


def check_args(argv):

    input_file = "grafcet_description"
    output_file = "grafcet_code"
    help_message = 'create_grafcet.py [-c] -i <inputfile(yaml filename,w/o extension)> ' \
                    '-o <outputfile(py/h&cpp filename,w/o extension)>'
    python_option = True
    try:
        opts, args = getopt.getopt(argv,"hci:o:", ["cpp", "ifile=", "ofile="])
    except getopt.GetoptError:
        print help_message
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print help_message
            sys.exit()
        elif opt in ("-i", "--ifile"):
            input_file = arg
        elif opt in ("-o", "--ofile"):
            output_file = arg
        elif opt in ("-c", "--cpp"):
            python_option = False

    return input_file, output_file, python_option


if __name__ == "__main__":

    arguments = check_args(sys.argv[1:])

    steps = []
    with open(arguments[0] + ".yaml", 'r') as stream:
        grafcet_description = yaml.load(stream)

    grafcet_id = grafcet_description[0]["prefix"]
    del grafcet_description[0]
    transitions = grafcet_description

    python = arguments[2]
    py_grafcet_file = None
    hpp_grafcet_file = None
    cpp_grafcet_file = None
    grafcet_name = grafcet_id + "Grafcet"


    if python is True:

        # PYTHON!
        print("About to create " + grafcet_id + " classes in Python!")
        py_grafcet_file = open(arguments[1] + ".py", "w")

        py_grafcet_file.write("#!/usr/bin/env python")

        txt = """

from forssea_utilities.grafcet import GrafcetStep
from forssea_utilities.grafcet import GrafcetTransition
from forssea_utilities.grafcet import GrafcetCondition
from forssea_utilities.grafcet import Grafcet
from forssea_utilities.grafcet import Action

    """
        py_grafcet_file.write(txt)

    else:

        # CPP!
        print("About to create " + grafcet_id + " classes in C++!")
        hpp_grafcet_file = open(arguments[1] + ".hpp", "w")

        txt = """
#ifndef FORSSEA_{grafcetName}_HPP
#define FORSSEA_{grafcetName}_HPP

#include <forssea_utilities/grafcet.hpp>
using namespace forssea_utilities;

    """
        txt = txt.format(grafcetName=grafcet_name.upper())
        hpp_grafcet_file.write(txt)

        cpp_grafcet_file = open(arguments[1] + ".cpp", "w")

        txt = """
#include <{file}.hpp>
    """
        txt = txt.format(file=arguments[1])
        cpp_grafcet_file.write(txt)

    # Generate the GrafcetStep / Transition / Condition classes
    created_steps = []
    for row in transitions:
        i_step_in_row = 0
        for step in row:

            # Check first if the step had been processed
            step_exists = False
            i_step = 0
            while not step_exists and i_step < len(created_steps):
                if created_steps[i_step] == step:
                    step_exists = True
                i_step += 1

            # Generate the step action class
            if not step_exists:

                class_name = grafcet_id + "S" + str(step)

                if python is True:

                    txt = """
class {className}(Action):
    def __init__(self, grafcet):
        super({className}, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    """
                    txt = txt.format(className=class_name)
                    py_grafcet_file.write(txt)

                else:

                    # hpp code
                    txt = """
class {className} : public Action
{{
public:
    {className}(Grafcet* grafcet) : Action(grafcet) {{ this->initialize(); }}
    virtual ~{className}() {{}}

    virtual bool execute();
}};
                    """
                    txt = txt.format(className=class_name)
                    hpp_grafcet_file.write(txt)

                    # cpp code
                    txt = """
bool {className}::execute()
{{
    return true;
}}
                    """
                    txt = txt.format(className=class_name)
                    cpp_grafcet_file.write(txt)

                print "Created Step {}".format(step)

            created_steps.append(step)

            # Generate the downstream transition condition classes
            if i_step_in_row < len(row)-1:

                next_step = row[i_step_in_row+1]
                transition_name = grafcet_id + "S" + str(step) + "S" + str(next_step)

                if python is True:

                    txt = """
class {transitionName}Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super({transitionName}Condition, self).__init__("{transitionName}", grafcet)

    def eval(self):
        # To be implemented
        return False

                    """
                    txt = txt.format(transitionName=transition_name)
                    py_grafcet_file.write(txt)

                else:

                    # hpp code
                    txt = """
class {transitionName}Condition : public GrafcetCondition
{{
public:
    {transitionName}Condition(Grafcet* grafcet) : GrafcetCondition(string("{transitionName}"), grafcet) {{}}
    virtual ~{transitionName}Condition() {{}}

    virtual bool eval();
}};
                    """
                    txt = txt.format(transitionName=transition_name)
                    hpp_grafcet_file.write(txt)

                    # cpp code
                    txt = """
bool {transitionName}Condition::eval()
{{
    return false;
}}
                    """
                    txt = txt.format(transitionName=transition_name)
                    cpp_grafcet_file.write(txt)

                print "Created Transition {}/{}".format(step, next_step)

            i_step_in_row += 1

    # Generate the Grafcet class
    if python is True:

        txt = """
class {grafcetName}(Grafcet):
    def __init__(self, frequency):
        super({grafcetName}, self).__init__("{grafcetName}", frequency)

    def _specific_setup(self):"""
        txt = txt.format(grafcetName=grafcet_name)
        py_grafcet_file.write(txt)

    else:

        txt = """
class {grafcetName}Grafcet : public Grafcet
{{
public:
    {grafcetName}Grafcet(ros::NodeHandle* node_handle, const float& frequency);
    virtual ~{grafcetName}Grafcet() {{}}

private:
    virtual void specificSetup();
}};

"""
        txt = txt.format(grafcetName=grafcet_name)
        hpp_grafcet_file.write(txt)

        txt = """
{grafcetName}Grafcet::{grafcetName}Grafcet(ros::NodeHandle* node_handle, const float& frequency)
: Grafcet(string("{grafcetName}Grafcet"), node_handle, frequency)
{{
    this->setup();
}}

void {grafcetName}Grafcet::specificSetup()
{{
    list<GrafcetTransition*> transitions;
    list<int> upstream_step_ids;
    list<int> downstream_step_ids;
    Action* action = 0;
    GrafcetStep* step = 0;
"""
        txt = txt.format(grafcetName=grafcet_name)
        cpp_grafcet_file.write(txt)

    created_steps = []
    for row in transitions:
        for step in row:
            # Check first if the step had been processed
            step_exists = False
            i_step = 0
            while not step_exists and i_step < len(created_steps):
                if created_steps[i_step] == step:
                    step_exists = True
                i_step += 1

            if not step_exists:

                created_steps.append(step)

                if python is True:

                    txt = """

        # Step {step:d}
        transitions = []
        action = {grafcetId}S{step:d}(self)
                    """
                    txt = txt.format(step=step, grafcetId=grafcet_id)
                    py_grafcet_file.write(txt)

                else:

                    txt = """
    // Step {step:d}
    transitions.clear();
    action = new {grafcetId}S{step:d}(this);
                    """
                    txt = txt.format(step=step, grafcetId=grafcet_id)
                    cpp_grafcet_file.write(txt)

                for i_step_list in range(0, len(transitions)):
                    for i_step in range(0, len(transitions[i_step_list])-1):
                        if transitions[i_step_list][i_step] == step:

                            if python is True:

                                txt = """
        # Transition S{upStep:d} / S{downStep:d} Condition
        upstream_step_ids = [{upStep:d}]
        downstream_step_ids = [{downStep:d}]
        transitions.append(GrafcetTransition({grafcetId}S{upStep:d}S{downStep:d}Condition(self), \
upstream_step_ids, downstream_step_ids))
                                """
                                txt = txt.format(upStep=transitions[i_step_list][i_step],
                                                 downStep=transitions[i_step_list][i_step+1], grafcetId=grafcet_id)
                                py_grafcet_file.write(txt)

                            else:

                                txt = """
    // Tansition S{upStep:d} / S{downStep:d} Condition
    upstream_step_ids.clear();
    downstream_step_ids.clear();
    upstream_step_ids.push_back({upStep:d});
    downstream_step_ids.push_back({downStep:d});
    transitions.push_back(new GrafcetTransition(new {grafcetId}S{upStep:d}S{downStep:d}Condition(), \
upstream_step_ids, downstream_step_ids));
                                """
                                txt = txt.format(upStep=transitions[i_step_list][i_step],
                                                 downStep=transitions[i_step_list][i_step+1], grafcetId=grafcet_id)
                                cpp_grafcet_file.write(txt)

                if python is True:

                    txt = """
        step_object = GrafcetStep({step:d}, transitions, action)
        step_object.set_active({active})
        self._add_step(step_object)"""

                    active = "True" if step == 0 else "False"
                    txt = txt.format(step=step, active=active)
                    py_grafcet_file.write(txt)

                else:

                    txt = """
    step = new GrafcetStep({step:d}, transitions, action);
    step->setActive({active});
    this->addStep(step);"""

                    active = "true" if step == 0 else "false"
                    txt = txt.format(step=step, active=active)
                    cpp_grafcet_file.write(txt)

    if python is True:

        py_grafcet_file.write("\n")
        py_grafcet_file.close()

    else:

        hpp_grafcet_file.write("#endif")
        hpp_grafcet_file.close()
        txt = """
}
"""
        cpp_grafcet_file.write(txt)
        cpp_grafcet_file.close()
