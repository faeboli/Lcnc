#!/usr/bin/env python3

import dearpygui.dearpygui as dpg
from gui.LcncPins import *

def RunGui(conf,args,configuration):

    dpg.create_context()
    
    input_lines=[]  #list of input item lines in the gui
    output_lines=[] #list of output item lines in the gui
    pwm_lines=[] #list of pwm item lines in the gui
    encoder_lines=[] #list of encoder dropdown in the gui
    encoder_lines_a=[] #list of encoder item lines for 'a' pins in the gui
    encoder_lines_b=[] #list of encoder item lines for 'b' pins in the gui
    stepgen_lines=[] #list of stepgen dropdown in the gui
    stepgen_lines_step=[] #list of stepgen item lines for 'step' pins in the gui
    stepgen_lines_dir=[] #list of stepgen item lines for 'dir' pins in the gui
    freepinslist=[] # free pins
    usedpinslist=[] # used pins
    usedpinslist=[] # used pins
    fileoper=''
    
    # add a peripheral: a line for single pin peripherals, or a dropdown + a series of lines for peripherals with more than a pin
    # the configuration is not updated here since the pin number will be unpopulated, the configuration is updated when the pin number is selected
    def add_item(sender,app_data):
        if(sender=='add_in_line'):
            input_lines.append(dpg.add_combo(freepinslist,label="input"+str(len(input_lines)),before="del_in_line",parent="input_list",callback=pin_update,width=70,indent=20))
        if(sender=='add_out_line'):
            output_lines.append(dpg.add_combo(freepinslist,label="output"+str(len(output_lines)),before="del_out_line",parent="output_list",callback=pin_update,width=70,indent=20))
        if(sender=='add_pwm_line'):
            pwm_lines.append(dpg.add_combo(freepinslist,label="pwm"+str(len(pwm_lines)),before="del_pwm_line",parent="pwm_list",callback=pin_update,width=70,indent=20))
        if(sender=='add_encoder_line'):
            encoder_lines.append(dpg.add_collapsing_header(label="Encoder"+str(len(encoder_lines)),before="del_encoder_line",parent="encoder_list",indent=20))
            encoder_lines_a.append(dpg.add_combo(freepinslist,label="input A",parent=encoder_lines[-1],callback=pin_update,width=70,indent=20))
            encoder_lines_b.append(dpg.add_combo(freepinslist,label="input B",parent=encoder_lines[-1],callback=pin_update,width=70,indent=20))
        if(sender=='add_stepgen_line'):
            stepgen_lines.append(dpg.add_collapsing_header(label="Stepgen"+str(len(stepgen_lines)),before="del_stepgen_line",parent="stepgen_list",indent=20))
            stepgen_lines_step.append(dpg.add_combo(freepinslist,label="Step out",parent=stepgen_lines[-1],callback=pin_update,width=70,indent=20))
            stepgen_lines_dir.append(dpg.add_combo(freepinslist,label="Dir out",parent=stepgen_lines[-1],callback=pin_update,width=70,indent=20))
        free_pins_update()
        combo_lists_update()
        pinshow_update()
    
    # deletes the last peripheral in the list from the bottom up
    # updates also the configuration
    def del_item(sender,app_data):
        if(sender=='del_in_line'):
            configuration['inputs'].remove(dpg.get_value(input_lines[-1]))
            dpg.delete_item(input_lines[-1])
            input_lines.pop(-1)
        if(sender=='del_out_line'):
            configuration['outputs'].remove(dpg.get_value(output_lines[-1]))
            dpg.delete_item(output_lines[-1])
            output_lines.pop(-1)
        if(sender=='del_pwm_line'):
            configuration['pwms'].remove(dpg.get_value(pwm_lines[-1]))
            dpg.delete_item(pwm_lines[-1])
            pwm_lines.pop(-1)
        if(sender=='del_encoder_line'):
            for i in configuration['encoders']: # removal is done on a pair basis
                if(i['a']==dpg.get_value(encoder_lines_a[-1])): # first is searched for the value
                    i.pop('a')  # removal of 'a' key and value
                    i.pop('b')  # removal of 'b' key and value
            configuration['encoders'].remove({}) # cleanup of null keys
            dpg.delete_item(encoder_lines[-1])
            encoder_lines.pop(-1)
            encoder_lines_a.pop(-1)
            encoder_lines_b.pop(-1)
        if(sender=='del_stepgen_line'):
            for i in configuration['stepgens']:
                if(i['step']==dpg.get_value(stepgen_lines_step[-1])):
                    i.pop('step')
                    i.pop('dir')
            configuration['stepgens'].remove({})
            dpg.delete_item(stepgen_lines[-1])
            stepgen_lines.pop(-1)
            stepgen_lines_step.pop(-1)
            stepgen_lines_dir.pop(-1)
        free_pins_update()
        combo_lists_update()
        pinshow_update()
        showconf_update()

    # update all the pins lists in the peripherals according to the freepinslist
    def combo_lists_update():
        dpg.configure_item('reset',items=freepinslist)
        if(configuration['board']=='5a-75b'):
            dpg.configure_item('revision',items=("6.1","7.0","8.0"))
        else:
            if(configuration['board']=='5a-75e'):
                dpg.configure_item('revision',items=("6.0","7.1"))
        for i in input_lines:
            dpg.configure_item(i,items=freepinslist)
        for i in output_lines:
            dpg.configure_item(i,items=freepinslist)
        for i in pwm_lines:
            dpg.configure_item(i,items=freepinslist)
        for i in encoder_lines_a:
            dpg.configure_item(i,items=freepinslist)
        for i in encoder_lines_b:
            dpg.configure_item(i,items=freepinslist)
        for i in stepgen_lines_step:
            dpg.configure_item(i,items=freepinslist)
        for i in stepgen_lines_dir:
            dpg.configure_item(i,items=freepinslist)

    # here the pin for each peripheral is selected and the configuration is updated
    def pin_update(sender,app_data):
        if(sender=='board'):
            configuration['board']=args.board=app_data
        if(sender=='revision'):
            configuration['revision']=args.revision=app_data
        if(sender=='clock'):
            configuration['clock']=args.sys_clk_freq=app_data
        if(sender=='reset'):
            configuration['reset']=app_data
        if(sender=='phy'):
            configuration['phy']=args.eth_phy=app_data
        if(sender=='ip'):
            configuration['ip']=args.eth_ip=app_data
        if(sender=='port'):
            configuration['port']=args.eth_port=app_data
        if(sender=='mac'):
            configuration['mac']=args.mac_address=app_data
        if(sender in input_lines):
            configuration['inputs'].clear()
            for i in input_lines:
                if(dpg.get_value(i)!=''):
                    configuration['inputs'].append(dpg.get_value(i))
        if(sender in output_lines):
            configuration['outputs'].clear()
            for i in output_lines:
                if(dpg.get_value(i)!=''):
                    configuration['outputs'].append(dpg.get_value(i))
        if(sender in pwm_lines):
            configuration['pwms'].clear()
            for i in pwm_lines:
                if(dpg.get_value(i)!=''):
                    configuration['pwms'].append(dpg.get_value(i))
        if(sender in encoder_lines_a or sender in encoder_lines_b):
            configuration['encoders'].clear()
            for i in range(len(encoder_lines_a)):
                if(dpg.get_value(encoder_lines_a[i])!='' and dpg.get_value(encoder_lines_b[i])!=''):
                    configuration['encoders'].append({'a':dpg.get_value(encoder_lines_a[i]),'b':dpg.get_value(encoder_lines_b[i])})
        if(sender in stepgen_lines_step or sender in stepgen_lines_dir):
            configuration['stepgens'].clear()
            for i in range(len(stepgen_lines_step)):
                if(dpg.get_value(stepgen_lines_step[i])!='' and dpg.get_value(stepgen_lines_dir[i])!=''):
                    configuration['stepgens'].append({'step':dpg.get_value(stepgen_lines_step[i]),'dir':dpg.get_value(stepgen_lines_dir[i])})
        free_pins_update()
        combo_lists_update()
        pinshow_update()
        showconf_update()

    # updates the free and used pin lists and their state in the pinout, starting from the configuration
    def free_pins_update():
        usedpinslist.clear()
        freepinslist.clear()
        # reset pin
        usedpinslist.append(configuration['reset'])
        # inputs
        for i in configuration['inputs']:
            usedpinslist.append(i)
        # outputs
        for i in configuration['outputs']:
            usedpinslist.append(i)
        # pwm
        for i in configuration['pwms']:
            usedpinslist.append(i)
        # encoders
        for i in configuration['encoders']:
            usedpinslist.append(i['a'])
            usedpinslist.append(i['b'])
        # stepgens
        for i in configuration['stepgens']:
            usedpinslist.append(i['step'])
            usedpinslist.append(i['dir'])
        # update pin status
        for i in pinout[configuration['board']]:
            for j in pinout[configuration['board']][i]:
                if(j in usedpinslist):
                    pinout[configuration['board']][i][j]='used'
                else:
                    pinout[configuration['board']][i][j]='free'
                    freepinslist.append(j)

    # update the pin list with free and not free pins and the matching color of the label
    def pinshow_update():
        dpg.delete_item('pins')
        dpg.add_group(tag='pins',parent='all_pins')
        for i in pinout[configuration['board']]:
            for j in pinout[configuration['board']][i]:
                pinstatus=pinout[configuration['board']][i][j]
                dpg.add_group(horizontal=True,indent=20,tag=j+'pins_group',parent='pins')
                dpg.add_text(default_value=j,parent=j+'pins_group')
                if(pinstatus=='free'):
                    dpg.add_text(tag=j,default_value=pinstatus,color=(0,255,0),parent=j+'pins_group')
                else:
                    dpg.add_text(tag=j,default_value=pinstatus,color=(255,0,0),parent=j+'pins_group')

    #removes all the unfilled items where the pins were not specified, leaves back garbled numbering of the items
    def items_clean():
        nonlocal input_lines
        nonlocal output_lines
        nonlocal pwm_lines
        nonlocal encoder_lines
        nonlocal encoder_lines_a
        nonlocal encoder_lines_b
        nonlocal stepgen_lines
        nonlocal stepgen_lines_step
        nonlocal stepgen_lines_dir
        tobedeleted=[]
        
        for i in input_lines:
            if(dpg.get_value(i)==''):
                dpg.delete_item(i)
                tobedeleted.append(i)
        for i in tobedeleted:
            input_lines.remove(i)
        tobedeleted.clear()
        for i in output_lines:
            if(dpg.get_value(i)==''):
                dpg.delete_item(i)
                tobedeleted.append(i)
        for i in tobedeleted:
            output_lines.remove(i)
        tobedeleted.clear()
        for i in pwm_lines:
            if(dpg.get_value(i)==''):
                dpg.delete_item(i)
                tobedeleted.append(i)
        for i in tobedeleted:
            pwm_lines.remove(i)
        tobedeleted.clear()
        for i in range(len(encoder_lines)):
            if(dpg.get_value(encoder_lines_a[i])=='' or dpg.get_value(encoder_lines_b[i])==''):
                dpg.delete_item(encoder_lines[i])
                tobedeleted.append(i)
        for i in tobedeleted:
            encoder_lines_a.pop(i)
            encoder_lines_b.pop(i)
            encoder_lines.pop(i)
        tobedeleted.clear()
        for i in range(len(stepgen_lines)):
            if(dpg.get_value(stepgen_lines_step[i])=='' or dpg.get_value(stepgen_lines_dir[i])==''):
                dpg.delete_item(stepgen_lines[i])
                tobedeleted.append(i)
        for i in tobedeleted:
            stepgen_lines_step.pop(i)
            stepgen_lines_dir.pop(i)
            stepgen_lines.pop(i)

    # delete all the peripherals from the gui
    def items_delete():
        nonlocal input_lines
        nonlocal output_lines
        nonlocal pwm_lines
        nonlocal encoder_lines
        nonlocal encoder_lines_a
        nonlocal encoder_lines_b
        nonlocal stepgen_lines
        nonlocal stepgen_lines_step
        nonlocal stepgen_lines_dir
        
        for i in input_lines:
            dpg.delete_item(i)
        input_lines=[]
        for i in output_lines:
            dpg.delete_item(i)
        output_lines=[]
        for i in pwm_lines:
            dpg.delete_item(i)
        pwm_lines=[]
        for i in encoder_lines:
            dpg.delete_item(i)
        encoder_lines=[]
        encoder_lines_a=[]
        encoder_lines_b=[]
        for i in stepgen_lines:
            dpg.delete_item(i)
        stepgen_lines=[]
        stepgen_lines_step=[]
        stepgen_lines_dir=[]

    # populates all the peripherals in the gui according to configuration
    def items_fill():
        for i in configuration['inputs']:
            add_item('add_in_line',0)
            dpg.configure_item(input_lines[-1],default_value=i)
        for i in configuration['outputs']:
            add_item('add_out_line',0)
            dpg.configure_item(output_lines[-1],default_value=i)
        for i in configuration['pwms']:
            add_item('add_pwm_line',0)
            dpg.configure_item(pwm_lines[-1],default_value=i)
        for i in configuration['encoders']:
            add_item('add_encoder_line',0)
            dpg.configure_item(encoder_lines_a[-1],default_value=i['a'])
            dpg.configure_item(encoder_lines_b[-1],default_value=i['b'])
        for i in configuration['stepgens']:
            add_item('add_stepgen_line',0)
            dpg.configure_item(stepgen_lines_step[-1],default_value=i['step'])
            dpg.configure_item(stepgen_lines_dir[-1],default_value=i['dir'])

    # shows the configuration raw dictionary
    def showconf_update():
        paragraphs=''
        for i in configuration:
            paragraphs+=str(i)+'='+str(configuration[i])+"\n"
        dpg.set_value('show_conf',paragraphs)

    # used to load or save the configuration from file
    def filedialog(sender,app_data):
        global fileoper
        if(fileoper=='load'):
            filename=app_data['selections'][list(app_data['selections'])[0]]
            print('Load from '+filename)
            args.loadconf=filename
            conf.importconf(args,configuration)
            items_delete()
            items_fill()
            free_pins_update()
            combo_lists_update()
            pinshow_update()
            showconf_update()
        if(fileoper=='save'):
            savefilename=app_data['file_path_name']
            savefilename=savefilename.rstrip('*')+'ini'
            print('Save to '+savefilename)
            items_clean()
            free_pins_update()
            combo_lists_update()
            pinshow_update()
            showconf_update()
            args.saveconf=savefilename
            conf.exportconf(args,configuration)

    # selects file operation
    def file_oper(sender,app_data):
        global fileoper
        if(sender=='openfile'):
            fileoper='load'
            dpg.show_item("filedialog_id")
        if(sender=='savefile'):
            fileoper='save'
            dpg.show_item("filedialog_id")

    # stops the gui and starts the firmware generation
    def generate():
        args.build=True
        dpg.stop_dearpygui()
        
    free_pins_update() # update the free pin list
    
    with dpg.file_dialog(label="File Dialog",modal=True, width=400, height=400, show=False, callback=filedialog ,tag="filedialog_id"):
        dpg.add_file_extension(".*")
    with dpg.window(tag="show_conf_win",label="Configuration Data", show=False, width=300, height=600, pos=[400,0]):
        dpg.add_text(tag='show_conf',wrap=300)

    dpg.create_viewport(title='Lcnc Configurator', width=700, height=600)    

    # creates initial window configuration
    with dpg.window(tag='Main',label="Configurator", width=400, height=600):
        with dpg.menu_bar():
            with dpg.menu(label="Menu"):
                dpg.add_menu_item(label="Open",callback=file_oper,tag='openfile')
                dpg.add_menu_item(label="Save As...",callback=file_oper,tag='savefile')
                dpg.add_menu_item(label="Generate and exit",callback=generate)
                dpg.add_menu_item(label="Show configuration",callback=lambda: dpg.show_item("show_conf_win"))
        with dpg.collapsing_header(label="Hardware"):
            dpg.add_combo(("5a-75e", "5a-75b"),default_value=configuration['board'],label="Board",tag='board',callback=pin_update,width=150,indent=20)
            dpg.add_combo(("6.0","6.1","7.0","7.1","8.0"),default_value=configuration['revision'],tag='revision',label="Revision",callback=pin_update,width=150,indent=20)
            dpg.add_input_float(default_value=configuration['clock'],label="clock",tag='clock',format="%f",callback=pin_update,width=150,indent=20)
            dpg.add_combo(freepinslist,default_value=configuration['reset'],label="ResetIn",tag='reset',callback=pin_update,width=150,indent=20)
            dpg.add_combo((0,1),default_value=configuration['phy'],label="Phy",tag='phy',callback=pin_update,width=150,indent=20)
            dpg.add_input_text(default_value=configuration['ip'],label="ip address",tag='ip',hint="enter address in the format 192.168.1.10",callback=pin_update,width=150,indent=20)
            dpg.add_input_int(default_value=configuration['port'],label="port",tag='port',callback=pin_update,width=150,indent=20)
            dpg.add_input_text(default_value=configuration['mac'],label="mac address",tag='mac',hint="enter mac address in the format 0x10e2d5000000",callback=pin_update,width=150,indent=20)
        with dpg.collapsing_header(label="Inputs",tag="input_list"):
            dpg.add_button(label="Add input",tag="add_in_line",callback=add_item,indent=20)
            dpg.add_button(label="Del input",tag="del_in_line",callback=del_item,indent=20)
        with dpg.collapsing_header(label="Outputs",tag="output_list"):
            dpg.add_button(label="Add output",tag="add_out_line",callback=add_item,indent=20)
            dpg.add_button(label="Del output",tag="del_out_line",callback=del_item,indent=20)
        with dpg.collapsing_header(label="Pwms",tag="pwm_list"):
            dpg.add_button(label="Add pwm",tag="add_pwm_line",callback=add_item,indent=20)
            dpg.add_button(label="Del pwm",tag="del_pwm_line",callback=del_item,indent=20)
        with dpg.collapsing_header(label="Encoders",tag="encoder_list"):
            dpg.add_button(label="Add encoder",tag="add_encoder_line",callback=add_item,indent=20)
            dpg.add_button(label="Del encoder",tag="del_encoder_line",callback=del_item,indent=20)
        with dpg.collapsing_header(label="Step Generators",tag="stepgen_list"):
            dpg.add_button(label="Add stepgen",tag="add_stepgen_line",callback=add_item,indent=20)
            dpg.add_button(label="Del stepgen",tag="del_stepgen_line",callback=del_item,indent=20)
        with dpg.collapsing_header(label="All pins",tag="all_pins"):
            dpg.add_group(tag='pins')

    # fills the gui with the peripherals in configuration
    items_fill()
    # updates the pins status list
    pinshow_update()
    # updates the show configuration window contents
    showconf_update()
    
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

if __name__ == "__main__":
    RunGui(args,configuration)
