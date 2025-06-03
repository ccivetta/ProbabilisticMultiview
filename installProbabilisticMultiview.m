function installProbabilisticMultiview(replaceExisting,skipAdmin)
% INSTALLPROBABILISTICMULTIVIEW installs Probabilistic Multiview tools for MATLAB.
%   INSTALLPROBABILISTICMULTIVIEW installs Probabilistic Multiview tools into the
%   following locations:
%                        Source: Destination
%     PlottingToolboxFunctions: matlabroot\toolbox\probabilitymultiviewdemo
%
%   INSTALLPROBABILISTICMULTIVIEW(true) installs Probabilistic Multiview tools
%   regardless of whether a copy of the Probabilistic Multiview tools exists
%   in the MATLAB root.
%
%   INSTALLPROBABILISTICMULTIVIEW(false) installs Probabilistic Multiview tools
%   only if no copy of the Probabilistic Multiview tools exists in the MATLAB
%   root.
%
%   M. Kutzer 02Jun2025, USNA

% Updates


%% Define support toolboxes
requiredToolboxes = {};

supportToolboxes = {};

%% Assign tool/toolbox specific parameters
dirName = 'probabilisticMultiview';
toolboxContent  = 'ProbabilisticMultiviewFunctions';
%toolboxExamples = 'ProbabilisticMultiview Example SCRIPTS';
toolboxName = 'Probabilistic Multiview';
toolboxShort = strrep(toolboxName, ' ', '');

%% Define toolbox directory options
toolboxPathAdmin = fullfile(matlabroot,'toolbox',dirName);
toolboxPathLocal = fullfile(prefdir,'toolbox',dirName);
toolboxPathExamples = fullfile(userpath,sprintf('Examples, %s',toolboxName));

%% Check if folders exist
isPathAdmin = isfolder(toolboxPathAdmin);
isPathLocal = isfolder(toolboxPathLocal);
isPathExamples = isfolder(toolboxPathExamples);

%% Check if folders are in the MATLAB path
allPaths = path;
allPaths = strsplit(allPaths,pathsep);

inPathAdmin = any(matches(allPaths,toolboxPathAdmin),'all');
inPathLocal = any(matches(allPaths,toolboxPathLocal),'all');
inPathExamples = any(matches(allPaths,toolboxPathExamples),'all');

%% Check inputs
if nargin < 1
    replaceExisting = [];
end
if nargin < 2
    skipAdmin = false;
end

%% Check for admin write access
isAdmin = checkWriteAccess(matlabroot);
isLocal = checkWriteAccess(prefdir);
isExample = checkWriteAccess(userpath);

%% Check for basic write access
if ~isLocal
    warning('No local write access?');
    return
end

%% Installation error solution(s)
adminSolution = sprintf(...
    ['Possible solution:\n',...
    '\t(1) Close current instance of MATLAB\n',...
    '\t(2) Open a new instance of MATLAB "as administrator"\n',...
    '\t\t(a) Locate MATLAB shortcut\n',...
    '\t\t(b) Right click\n',...
    '\t\t(c) Select "Run as administrator"\n']);

%% Prompt user if not an admin
if ~isAdmin && ~skipAdmin
    choice = questdlg(sprintf(...
        ['MATLAB is running without administrative privileges.\n',...
        'Would you like to install the %s locally?'],toolboxName),...
        sprintf('Local Install %s',toolboxName),...
        'Yes','No','Cancel','Yes');
    % Replace existing or cancel installation
    switch choice
        case 'Yes'
            skipAdmin = true;
        case 'No'
            fprintf('Unable to write perform installation.\n\n');
            fprintf('To install as an administrator - %s',adminSolution);
            return
        case 'Cancel'
            fprintf('Action cancelled.\n');
            return
        otherwise
            error('Unexpected response.');
    end
end

%% Check for existing toolbox
if skipAdmin
    isToolbox = isPathLocal;
    toolboxPath = toolboxPathLocal;
else
    isToolbox = isPathAdmin;
    toolboxPath = toolboxPathAdmin;
end

% Support contents
if exist('toolboxSupport','var')
    supportPath = fullfile(toolboxPath,toolboxSupport);
end

%% Check for toolbox directory
if isToolbox
    % Apply replaceExisting argument
    if isempty(replaceExisting)
        choice = questdlg(sprintf(...
            ['MATLAB path already contains the %s.\n',...
            'Would you like to replace the existing toolbox?'],toolboxName),...
            sprintf('Replace Existing %s',toolboxName),...
            'Yes','No','Cancel','Yes');
    elseif replaceExisting
        choice = 'Yes';
    else
        choice = 'No';
    end
    % Replace existing or cancel installation
    switch choice
        case 'Yes'
            % Remove existing paths
            removePath(toolboxName,...
                toolboxPathAdmin,inPathAdmin,isPathAdmin,isAdmin);
            removePath(toolboxName,...
                toolboxPathLocal,inPathLocal,isPathLocal,isLocal);
            removePath(toolboxName,...
                toolboxPathExamples,inPathExamples,inPathExamples,isExample);
        case 'No'
            fprintf('%s currently exists, installation cancelled.\n',toolboxName);
            return
        case 'Cancel'
            fprintf('Action cancelled.\n');
            return
        otherwise
            error('Unexpected response.');
    end
end

%% Migrate toolbox folder contents
% Toolbox contents
migrateContent(toolboxContent,toolboxPath,toolboxName);
% Support contents
if exist('toolboxSupport','var')
    migrateContent(toolboxSupport,supportPath,...
        sprintf('%s Support',toolboxName));
end
% Example files
try
    migrateContent(toolboxExamples,toolboxPathExamples,...
        sprintf('%s Examples',toolboxName));
catch
    fprintf('Unable to migrate examples.\n');
end

%% Save toolbox path
addpath(genpath(toolboxPath),'-end'); % Add path with subdirectories
%addpath(toolboxPath,'-end');         % Add path only

pathdef_local = fullfile(userpath,'pathdef.m');
if isAdmin
    % Delete local pathdef file
    if isfile(pathdef_local)
        delete(pathdef_local);
    end
    % Save administrator local pathdef file
    savepath;
else
    % Create local user pathdef file
    fprintf('Updating local user "pathdef.m"...')
    savepath( pathdef_local );
    fprintf('[Complete]\n');
end

%% Rehash toolbox cache
fprintf('Rehashing Toolbox Cache...');
rehash TOOLBOXCACHE
fprintf('[Complete]\n');

%% Install/Update required toolboxes
for ii = 1:size(requiredToolboxes,1)
    try
        ToolboxUpdate(requiredToolboxes{ii,:});
    catch ME
        fprintf(2,'[ERROR]\nUnable to install required toolbox: "%s"\n',requiredToolboxes{ii});
        fprintf(2,'\t%s\n',ME.message);
    end
end

for ii = 1:size(supportToolboxes,1)
    try
        SupportUpdate(supportToolboxes{ii,:});
    catch ME
        fprintf(2,'[ERROR]\nUnable to install required package: "%s"\n',supportToolboxes{ii});
        fprintf(2,'\t%s\n',ME.message);
    end
end


%% internal functions (shared workspace)
% ------------------------------------------------------------------------
    function migrateContent(sourceIn,destination,msg)

        % Migrate toolbox folder contents
        if ~isfolder(sourceIn)
            error(sprintf(...
                ['Change your working directory to the location of "install%s.m".\n',...
                '\n',...
                'If this problem persists:\n',...
                '\t(1) Unzip your original download of "%s" into a new directory\n',...
                '\t(2) Open a new instance of MATLAB "as administrator"\n',...
                '\t\t(a) Locate MATLAB shortcut\n',...
                '\t\t(b) Right click\n',...
                '\t\t(c) Select "Run as administrator"\n',...
                '\t(3) Change your "working directory" to the location of "install%s.m"\n',...
                '\t(4) Enter "install%s" (without quotes) into the command window\n',...
                '\t(5) Press Enter.'],toolboxShort,toolboxShort,toolboxShort,toolboxShort));
        end

        % Create Toolbox Path
        [isDir,msgDir,msgID] = mkdir(destination);
        if isDir
            fprintf('%s folder created successfully:\n\t"%s"\n',msg,destination);
        else
            fprintf('Failed to create %s folder:\n\t"%s"\n',msg,destination);
            fprintf(adminSolution);
            error(msgID,msgDir);
        end

        % Migrate contents
        files = dir(sourceIn);
        wb = waitbar(0,sprintf('Copying %s contents...',msg));
        n = numel(files);
        fprintf('Copying %s contents:\n',msg);
        for i = 1:n
            % source file location
            source = fullfile(sourceIn,files(i).name);

            if files(i).isdir
                switch files(i).name
                    case '.'
                        %Ignore
                    case '..'
                        %Ignore
                    otherwise
                        fprintf('\t%s...',files(i).name);
                        nDestination = fullfile(destination,files(i).name);
                        [isDir,msg,msgID] = mkdir(nDestination);
                        if isDir
                            [isCopy,msg,msgID] = copyfile(source,nDestination,'f');
                            if isCopy
                                fprintf('[Complete]\n');
                            else
                                bin = msg == char(10);
                                msg(bin) = [];
                                bin = msg == char(13);
                                msg(bin) = [];
                                fprintf('[Failed: "%s"]\n',msg);
                            end
                        else
                            bin = msg == char(10);
                            msg(bin) = [];
                            bin = msg == char(13);
                            msg(bin) = [];
                            fprintf('[Failed: "%s"]\n',msg);
                        end
                end
            else
                fprintf('\t%s...',files(i).name);
                [isCopy,msg,msgID] = copyfile(source,destination,'f');

                if isCopy == 1
                    fprintf('[Complete]\n');
                else
                    bin = msg == char(10);
                    msg(bin) = [];
                    bin = msg == char(13);
                    msg(bin) = [];
                    fprintf('[Failed: "%s"]\n',msg);
                end
            end
            waitbar(i/n,wb);
        end
        set(wb,'Visible','off');
        delete(wb);

    end

end

%% Internal functions (unique workspace)
% ------------------------------------------------------------------------
function tfWrite = checkWriteAccess(pname)

tmpFname = fullfile(pname,'tmp.txt');
tmpHndle = fopen(tmpFname, 'w');
if tmpHndle < 0
    tfWrite = false;
else
    tfWrite = true;
    fclose(tmpHndle);
    delete(tmpFname);
end

end
% ------------------------------------------------------------------------
function removePath(toolboxName,pName,inPath,isPath,isDelete)
% Remove path
if inPath
    rmpath(pName);
    fprintf('%s path removed successfully:\n\t"%s"\n',toolboxName,pName);

    % Check for subdirectories
    allPaths = path;
    allPaths = strsplit(allPaths,pathsep);
    % Remove subdirectories
    idxInPath = find( contains(allPaths,pName) );
    for i = reshape(idxInPath,1,[])
        sPath = allPaths{i};
        rmpath(sPath);
        fprintf(' -> %s subpath removed successfully:\n\t\t"%s"\n',toolboxName,sPath);
    end
end
% Remove folder
if isPath && isDelete
    [isRemoved, msg, msgID] = rmdir(pName,'s');
    if isRemoved
        fprintf('Previous version of %s removed successfully:\n\t"%s"\n',toolboxName,pName);
    else
        fprintf('Failed to remove old %s folder:\n\t"%s"\n',toolboxName,pName);
        %fprintf(adminSolution);
        error(msgID,msg);
    end
elseif ~isDelete
    fprintf('Skipping removal of old %s folder:\n\t"%s"\n',toolboxName,pName);
end

end
% ------------------------------------------------------------------------
function ToolboxUpdate(tbName,tbOrg,tbBranches)

% Set defaults
if nargin < 2
    tbOrg = 'kutzer';
end
if nargin < 3
    tbBranches = {'main','master'};
end
if isempty(tbBranches)
    tbBranches = {'main','master'};
end
if ~iscell(tbBranches)
    tbBranches = {tbBranches};
end

% Setup functions
ToolboxVer = str2func( sprintf('%sToolboxVer',tbName) );
installToolbox = str2func( sprintf('install%sToolbox',tbName) );

% Check current version
try
    A = ToolboxVer;
catch ME
    A = [];
    fprintf('No previous version of %s detected.\n',tbName);
end

% Setup temporary file directory
%fprintf('Downloading the %s Toolbox...',toolboxName);
tmpFolder = sprintf('%s',tbName);
pname = fullfile(tempdir,tmpFolder);
if isfolder(pname)
    % Remove existing directory
    [ok,msg] = rmdir(pname,'s');
end
% Create new directory
[ok,msg] = mkdir(tempdir,tmpFolder);

% Download and unzip toolbox (GitHub)
% UPDATED: 07Sep2021, M. Kutzer
%url = sprintf('https://github.com/kutzer/%sToolbox/archive/master.zip',toolboxName); <--- Github removed references to "master"
%url = sprintf('https://github.com/kutzer/%sToolbox/archive/refs/heads/main.zip',toolboxName);

% Check possible branches
%defBranches = {'master','main'};
for i = 1:numel(tbBranches)
    % Check default branch
    tbBranch = tbBranches{i};
    url = sprintf('https://github.com/%s/%sToolbox/archive/refs/heads/%s.zip',...
        tbOrg,tbName,tbBranch);

    % Download and unzip repository
    fprintf('Downloading the %s Toolbox ("%s" branch)...',tbName,tbBranch);
    try
        %fnames = unzip(url,pname);
        %urlwrite(url,fullfile(pname,tmpFname));
        tmpFname = sprintf('%sToolbox-master.zip',tbName);
        websave(fullfile(pname,tmpFname),url);
        fnames = unzip(fullfile(pname,tmpFname),pname);
        delete(fullfile(pname,tmpFname));

        fprintf('SUCCESS\n');
        confirm = true;
        break
    catch ME
        fprintf('"%s" branch does not exist\n',tbBranch);
        confirm = false;
        %fprintf(2,'ERROR MESSAGE:\n\t%s\n',ME.message);
    end
end

% Check for successful download
alternativeInstallMsg = [...
    sprintf('Manually download the %s Toolbox using the following link:\n',tbName),...
    newline,...
    sprintf('%s\n',url),...
    newline,...
    sprintf('Once the file is downloaded:\n'),...
    sprintf('\t(1) Unzip your download of the "%sToolbox"\n',tbName),...
    sprintf('\t(2) Change your "working directory" to the location of "install%sToolbox.m"\n',tbName),...
    sprintf('\t(3) Enter "install%sToolbox" (without quotes) into the command window\n',tbName),...
    sprintf('\t(4) Press Enter.')];

if ~confirm
    warning('InstallToolbox:FailedDownload','Failed to download updated version of %s Toolbox.',tbName);
    fprintf(2,'\n%s\n',alternativeInstallMsg);

    msgbox(alternativeInstallMsg, sprintf('Failed to download %s Toolbox',tbName),'warn');
    return
end

% Find base directory
install_pos = strfind(fnames, sprintf('install%sToolbox.m',tbName) );
sIdx = cell2mat( install_pos );
cIdx = ~cell2mat( cellfun(@isempty,install_pos,'UniformOutput',0) );

pname_star = fnames{cIdx}(1:sIdx-1);

% Get current directory and temporarily change path
cpath = cd;
cd(pname_star);

% Check for admin
skipAdmin = ~checkWriteAccess(matlabroot);

% Install Toolbox
% TODO - consider providing the user with an option or more information
%        related to "skipAdmin"
try
    installToolbox(true,skipAdmin);
catch ME
    cd(cpath);
    throw(ME);
end

% Move back to current directory and remove temp file
cd(cpath);
[ok,msg] = rmdir(pname,'s');
if ~ok
    warning('Unable to remove temporary download folder. %s',msg);
end

% Complete installation
fprintf('Installation complete.\n');

end
% ------------------------------------------------------------------------
function SupportUpdate(tbName,tbOrg,tbBranches)

% Set defaults
if nargin < 2
    tbOrg = 'kutzer';
end
if nargin < 3
    tbBranches = {'main','master'};
end
if isempty(tbBranches)
    tbBranches = {'main','master'};
end
if ~iscell(tbBranches)
    tbBranches = {tbBranches};
end

% Setup functions
ToolboxVer = str2func( sprintf('%sVer',tbName) );
installToolbox = str2func( sprintf('install%s',tbName) );

% Check current version
try
    A = ToolboxVer;
catch ME
    A = [];
    fprintf('No previous version of %s detected.\n',tbName);
end

% Setup temporary file directory
%fprintf('Downloading the %s ...',toolboxName);
tmpFolder = sprintf('%s',tbName);
pname = fullfile(tempdir,tmpFolder);
if isfolder(pname)
    % Remove existing directory
    [ok,msg] = rmdir(pname,'s');
end
% Create new directory
[ok,msg] = mkdir(tempdir,tmpFolder);

% Download and unzip toolbox (GitHub)
% UPDATED: 07Sep2021, M. Kutzer
%url = sprintf('https://github.com/kutzer/%s/archive/master.zip',toolboxName); <--- Github removed references to "master"
%url = sprintf('https://github.com/kutzer/%s/archive/refs/heads/main.zip',toolboxName);

% Check possible branches
%defBranches = {'master','main'};
for i = 1:numel(tbBranches)
    % Check default branch
    tbBranch = tbBranches{i};
    url = sprintf('https://github.com/%s/%s/archive/refs/heads/%s.zip',...
        tbOrg,tbName,tbBranch);

    % Download and unzip repository
    fprintf('Downloading the %s Package ("%s" branch)...',tbName,tbBranch);
    try
        %fnames = unzip(url,pname);
        %urlwrite(url,fullfile(pname,tmpFname));
        tmpFname = sprintf('%s-master.zip',tbName);
        websave(fullfile(pname,tmpFname),url);
        fnames = unzip(fullfile(pname,tmpFname),pname);
        delete(fullfile(pname,tmpFname));

        fprintf('SUCCESS\n');
        confirm = true;
        break
    catch ME
        fprintf('"%s" branch does not exist\n',tbBranch);
        confirm = false;
        %fprintf(2,'ERROR MESSAGE:\n\t%s\n',ME.message);
    end
end

% Check for successful download
alternativeInstallMsg = [...
    sprintf('Manually download the %s Toolbox using the following link:\n',tbName),...
    newline,...
    sprintf('%s\n',url),...
    newline,...
    sprintf('Once the file is downloaded:\n'),...
    sprintf('\t(1) Unzip your download of the "%sToolbox"\n',tbName),...
    sprintf('\t(2) Change your "working directory" to the location of "install%sToolbox.m"\n',tbName),...
    sprintf('\t(3) Enter "install%sToolbox" (without quotes) into the command window\n',tbName),...
    sprintf('\t(4) Press Enter.')];

if ~confirm
    warning('InstallToolbox:FailedDownload','Failed to download updated version of %s Toolbox.',tbName);
    fprintf(2,'\n%s\n',alternativeInstallMsg);

    msgbox(alternativeInstallMsg, sprintf('Failed to download %s Toolbox',tbName),'warn');
    return
end

% Find base directory
install_pos = strfind(fnames, sprintf('install%s.m',tbName) );
sIdx = cell2mat( install_pos );
cIdx = ~cell2mat( cellfun(@isempty,install_pos,'UniformOutput',0) );

pname_star = fnames{cIdx}(1:sIdx-1);

% Get current directory and temporarily change path
cpath = cd;
cd(pname_star);

% Check for admin
skipAdmin = ~checkWriteAccess(matlabroot);

% Install Toolbox
% TODO - consider providing the user with an option or more information
%        related to "skipAdmin"
try
    installToolbox(true,skipAdmin);
catch ME
    cd(cpath);
    throw(ME);
end

% Move back to current directory and remove temp file
cd(cpath);
[ok,msg] = rmdir(pname,'s');
if ~ok
    warning('Unable to remove temporary download folder. %s',msg);
end

% Complete installation
fprintf('Installation complete.\n');

end
