<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="12008004">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="PBLV_Interface.vi" Type="VI" URL="../PBLV_Interface.vi"/>
		<Item Name="PBLV_Example1.vi" Type="VI" URL="../PBLV_Example1.vi"/>
		<Item Name="PBLV_Example_SOS-Loop.vi" Type="VI" URL="../PBLV_Example_SOS-Loop.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="DialogTypeEnum.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/DialogTypeEnum.ctl"/>
				<Item Name="Check Special Tags.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Check Special Tags.vi"/>
				<Item Name="Set String Value.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set String Value.vi"/>
				<Item Name="GetRTHostConnectedProp.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/GetRTHostConnectedProp.vi"/>
				<Item Name="Error Code Database.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Code Database.vi"/>
				<Item Name="whitespace.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/whitespace.ctl"/>
				<Item Name="Trim Whitespace.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Trim Whitespace.vi"/>
				<Item Name="Format Message String.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Format Message String.vi"/>
				<Item Name="Set Bold Text.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Set Bold Text.vi"/>
				<Item Name="ErrWarn.ctl" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/ErrWarn.ctl"/>
				<Item Name="eventvkey.ctl" Type="VI" URL="/&lt;vilib&gt;/event_ctls.llb/eventvkey.ctl"/>
				<Item Name="Not Found Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Not Found Dialog.vi"/>
				<Item Name="Simple Error Handler.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Simple Error Handler.vi"/>
			</Item>
			<Item Name="pb_init.vi" Type="VI" URL="../pb_init.vi"/>
			<Item Name="pb_error_handler.vi" Type="VI" URL="../pb_error_handler.vi"/>
			<Item Name="pb_close.vi" Type="VI" URL="../pb_close.vi"/>
			<Item Name="pb_start_programming.vi" Type="VI" URL="../pb_start_programming.vi"/>
			<Item Name="pb_inst_pbonly.vi" Type="VI" URL="../pb_inst_pbonly.vi"/>
			<Item Name="pb_stop_programming.vi" Type="VI" URL="../pb_stop_programming.vi"/>
			<Item Name="pb_start.vi" Type="VI" URL="../pb_start.vi"/>
			<Item Name="pb_stop.vi" Type="VI" URL="../pb_stop.vi"/>
			<Item Name="pb_select_board.vi" Type="VI" URL="../pb_select_board.vi"/>
			<Item Name="General Error Handler MOD.vi" Type="VI" URL="../General Error Handler MOD.vi"/>
			<Item Name="Details Display Dialog-MOD.vi" Type="VI" URL="../Details Display Dialog-MOD.vi"/>
			<Item Name="pb_core_clock.vi" Type="VI" URL="../pb_core_clock.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="PBLV_Interface" Type="EXE">
				<Property Name="App_INI_aliasGUID" Type="Str">{C61206DC-5E47-4FC8-A797-6F8DA53F03BF}</Property>
				<Property Name="App_INI_GUID" Type="Str">{35DF4FB6-DB9B-4AED-B2EF-5A2C8E700C8A}</Property>
				<Property Name="App_winsec.description" Type="Str">http://www.Hewlett-Packard Company.com</Property>
				<Property Name="Bld_buildCacheID" Type="Str">{5F6B6C49-CF95-4D5E-80A6-9AE38A70261B}</Property>
				<Property Name="Bld_buildSpecName" Type="Str">PBLV_Interface</Property>
				<Property Name="Bld_excludeLibraryItems" Type="Bool">true</Property>
				<Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
				<Property Name="Bld_localDestDir" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Interface</Property>
				<Property Name="Bld_localDestDirType" Type="Str">relativeToCommon</Property>
				<Property Name="Bld_modifyLibraryFile" Type="Bool">true</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{86AE9571-DDDA-4CD7-A95C-BC7FA14A6264}</Property>
				<Property Name="Bld_targetDestDir" Type="Path"></Property>
				<Property Name="Destination[0].destName" Type="Str">PBLV_Interface.exe</Property>
				<Property Name="Destination[0].path" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Interface/PBLV_Interface.exe</Property>
				<Property Name="Destination[0].preserveHierarchy" Type="Bool">true</Property>
				<Property Name="Destination[0].type" Type="Str">App</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Interface/data</Property>
				<Property Name="DestinationCount" Type="Int">2</Property>
				<Property Name="Source[0].itemID" Type="Str">{0FFB39FC-10AE-4FBE-9AAD-FFD7E2481757}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/PBLV_Interface.vi</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">TopLevel</Property>
				<Property Name="Source[1].type" Type="Str">VI</Property>
				<Property Name="SourceCount" Type="Int">2</Property>
				<Property Name="TgtF_companyName" Type="Str">Hewlett-Packard Company</Property>
				<Property Name="TgtF_fileDescription" Type="Str">PBLV_Interface</Property>
				<Property Name="TgtF_fileVersion.major" Type="Int">1</Property>
				<Property Name="TgtF_internalName" Type="Str">PBLV_Interface</Property>
				<Property Name="TgtF_legalCopyright" Type="Str">Copyright © 2009 Hewlett-Packard Company</Property>
				<Property Name="TgtF_productName" Type="Str">PBLV_Interface</Property>
				<Property Name="TgtF_targetfileGUID" Type="Str">{FD048890-112F-427A-B718-E163E63F8B2A}</Property>
				<Property Name="TgtF_targetfileName" Type="Str">PBLV_Interface.exe</Property>
			</Item>
			<Item Name="PBLV_Example1" Type="EXE">
				<Property Name="App_INI_aliasGUID" Type="Str">{D34BEEA4-1112-4935-A747-0C17AE4ABC6F}</Property>
				<Property Name="App_INI_GUID" Type="Str">{34D15A1C-3130-47C8-96DD-00D88181B006}</Property>
				<Property Name="App_winsec.description" Type="Str">http://www.Hewlett-Packard Company.com</Property>
				<Property Name="Bld_buildCacheID" Type="Str">{9312EF42-7A40-43CD-82B3-12E5EFF84C56}</Property>
				<Property Name="Bld_buildSpecName" Type="Str">PBLV_Example1</Property>
				<Property Name="Bld_excludeLibraryItems" Type="Bool">true</Property>
				<Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
				<Property Name="Bld_localDestDir" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Example1</Property>
				<Property Name="Bld_localDestDirType" Type="Str">relativeToCommon</Property>
				<Property Name="Bld_modifyLibraryFile" Type="Bool">true</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{1BEF84F2-39E3-4EC4-92FB-1B6B855E601E}</Property>
				<Property Name="Bld_targetDestDir" Type="Path"></Property>
				<Property Name="Destination[0].destName" Type="Str">PBLV_Example1.exe</Property>
				<Property Name="Destination[0].path" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Example1/PBLV_Example1.exe</Property>
				<Property Name="Destination[0].preserveHierarchy" Type="Bool">true</Property>
				<Property Name="Destination[0].type" Type="Str">App</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">../builds/NI_AB_PROJECTNAME/PBLV_Example1/data</Property>
				<Property Name="DestinationCount" Type="Int">2</Property>
				<Property Name="Source[0].itemID" Type="Str">{0FFB39FC-10AE-4FBE-9AAD-FFD7E2481757}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/PBLV_Example1.vi</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">TopLevel</Property>
				<Property Name="Source[1].type" Type="Str">VI</Property>
				<Property Name="SourceCount" Type="Int">2</Property>
				<Property Name="TgtF_companyName" Type="Str">Hewlett-Packard Company</Property>
				<Property Name="TgtF_fileDescription" Type="Str">PBLV_Example1</Property>
				<Property Name="TgtF_fileVersion.major" Type="Int">1</Property>
				<Property Name="TgtF_internalName" Type="Str">PBLV_Example1</Property>
				<Property Name="TgtF_legalCopyright" Type="Str">Copyright © 2009 Hewlett-Packard Company</Property>
				<Property Name="TgtF_productName" Type="Str">PBLV_Example1</Property>
				<Property Name="TgtF_targetfileGUID" Type="Str">{B2E3A864-E2EE-43C5-82B9-257D1B558499}</Property>
				<Property Name="TgtF_targetfileName" Type="Str">PBLV_Example1.exe</Property>
			</Item>
		</Item>
	</Item>
</Project>
