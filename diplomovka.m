
function varargout = diplomovka(varargin)
% DIPLOMOVKA MATLAB code for diplomovka.fig
%      DIPLOMOVKA, by itself, creates a new DIPLOMOVKA or raises the existing
%      singleton*.
%
%      H = DIPLOMOVKA returns the handle to a new DIPLOMOVKA or the handle to
%      the existing singleton*.
%
%      DIPLOMOVKA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIPLOMOVKA.M with the given input arguments.
%
%      DIPLOMOVKA('Property','Value',...) creates a new DIPLOMOVKA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before diplomovka_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to diplomovka_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help diplomovka

% Last Modified by GUIDE v2.5 13-Apr-2018 14:27:57

% Begin initialization code - DO NOT EDIT

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @diplomovka_OpeningFcn, ...
                   'gui_OutputFcn',  @diplomovka_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before diplomovka is made visible.
function diplomovka_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to diplomovka (see VARARGIN)

% Choose default command line output for diplomovka


handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes diplomovka wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = diplomovka_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global person

% Choose person to find
folder_name = uigetdir('','Select Directory to Open');

if ~isequal(folder_name,0)  
    fdir = dir(folder_name);
    ridx = randi(numel(fdir));
    disp( ['Chosen file is: ' fdir(ridx).name] );
    File = fullfile(folder_name, fdir(ridx).name);
    image  = imread(File);
    axes(handles.axes1);
    imshow(image);
    
    out=regexp(folder_name,'\','split');
    person = out{length(out)};
    
end    


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoF extract folder_name2

%Create frames out of movie
[filename pathname] = uigetfile({'*.mp4';'*.avi';'*.*'}, 'File selector');
if ~ischar(filename)
    return;   %user canceled dialog
end

File = fullfile(pathname, filename);

try
    obj = VideoReader(File);
catch
    warndlg( 'File named in edit box does not appear to be a usable movie file');
    return
end

mkdir('VideoToFrames'); 
numFrames = obj.NumberOfFrames;
n=numFrames;
for i = 1:n
    frame = readframe(obj,i);
    imwrite(frame,['VideoToFrames\Image' int2str(i), '.jpg']);
    im(i)=image(frame);
end




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoF extract folder_name2 video_im

% Choose video in which to look for person
folder_name2 = uigetdir('','Select Directory to Open');
if ~isequal(folder_name2,0)  
    fdir2 = dir(folder_name2);
    
    ridx = randi(numel(fdir2));
    disp( ['Chosen file is: ' fdir2(ridx).name] );
    File = fullfile(folder_name2, fdir2(ridx).name);
    image  = imread(File);
    video_im = image; 
    axes(handles.axes2);
    imshow(video_im);
    
   
    
end    



% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoF extract folder_name2

%Create movie out of frames
pathname = uigetdir('','Select Directory to Open');
if ~isequal(pathname,0)
    
    fdir = dir(pathname);
    oldPath = cd;
    cd('FramesToVideo');
    writerObj = VideoWriter('Movie.avi');
    open(writerObj);
    for i = 1 : length(fdir)
        try
            fullFileName = fullfile(pathname, fdir(i).name);
            img = imread(fullFileName);    
            writeVideo(writerObj, img);
        catch
        end    
   
    end
    
    close(writerObj);
    cd(oldPath); 
   
end



% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global videoF extract folder_name2 faceClassifier faceDatabase databasePath personIndex person video_im

% Start recognition of a person in the video

% A video must be chosen first
try
    fdir2 = dir(folder_name2);
catch
    warndlg( 'Choose a video frames directory first');
    return
end

disp(person);

% A person must be chosen first
if isempty(person)
    disp('Choose a person to look for first');
else
    
poc = 0;
time_pom = 0;
found = 0;
flag = 0;
extr = 0;

% Searching through the set of video frames
for i=1:length(fdir2)
      
    try
        filename = fullfile(folder_name2,fdir2(i).name);
        Im = imread(filename);
        %axes(handles.axes2);
        %imshow(Im);
        
        faceDetector = vision.CascadeObjectDetector('MinSize',[50,50]);
       
        % Detecting face in the Im.
        bbox= step(faceDetector,Im);
        
        % Plot a rectangle around the detected face.
        Im = insertShape(Im,'Rectangle', bbox);
        
        %figure;
        %imshow(Im);%show image
        %title('Detected image');
        
        guidata(hObject, handles);%update data
        
        % Normalization of detected face
        for i=1:size(bbox,1)
            handles.img2 = imcrop(Im,bbox(i,:));
            scaleFactor = 96/size(handles.img2, 1);
            handles.img2 = imresize(handles.img2,scaleFactor);
            handles.img2 = rgb2gray(handles.img2);          
            %figure;
            %imshow(handles.img2);    
            %title('Cropped image');
            
            % Recognise a person using HOG features
            if extract == "hog"
                
                poc = poc+1;
                       
                start = tic;
                
                % Extract features
                queryFeatures = extractHOGFeatures(handles.img2);
                
                hogextr = toc(start);
                extr = extr + hogextr;
                
                Tstart = tic;
                
                % Predicted person by classificator
                personLabel = predict(faceClassifier,queryFeatures);
                
                tElapsed = toc(Tstart);
                time_pom = time_pom + tElapsed;

                % Map back to faceDatabase set to find identity 
                booleanIndex = strcmp(personLabel, personIndex);
                integerIndex = find(booleanIndex);
                
                
                personLabel =  string(personLabel);
                person = string(person);
                
                %figure;
                
                %subplot(1,2,1);imshow(handles.img2);title('Query Face');
                if  strcmp(personLabel,person)
                    found = found+1
                    %subplot(1,2,2);imshow(read(faceDatabase(integerIndex),1));title('Found');
                else
                    %subplot(1,2,2);imshow(read(faceDatabase(integerIndex),1));title('Matched face');
                end
                
                % Found the person for the fifth time
                if found == 5
                    axes(handles.axes2);
                    imshow(Im);title('Found');
                    flag = 1;
                    break;
                end
               
                
            end
            
            % Recognise a person using LBP features
            if extract == "lbp"
                
                poc = poc+1;
                               
                start = tic;
                
                % Extract features
                queryFeatures = extractLBPFeatures(handles.img2);
                
                hogextr = toc(start);
                extr = extr + hogextr;
                
                Tstart = tic;
                
                % Predicted person by classificator
                personLabel = predict(faceClassifier,queryFeatures);
                
                tElapsed = toc(Tstart);
                time_pom = time_pom + tElapsed;
                
                % Map back to faceDatabase set to find identity 
                booleanIndex = strcmp(personLabel, personIndex);
                integerIndex = find(booleanIndex);
                
                personLabel =  string(personLabel);
                person = string(person);
                           
                %figure;
                
                %subplot(1,2,1);imshow(handles.img2);title('Query Face');
                if  strcmp(personLabel,person)
                    found = found+1
                    %subplot(1,2,2);imshow(read(faceDatabase(integerIndex),1));title('Found');
                else
                    %subplot(1,2,2);imshow(read(faceDatabase(integerIndex),1));title('Matched face');
                end
                
                % Found the person for the fifth time 
                if found == 5
                    axes(handles.axes2);
                    imshow(Im);title('Found');
                    flag = 1;
                    break;
                end
                        
            end
                     
            % Recognise a person using Fisherface features
            if extract == "fisher"
                
                % Create a vector out of image
                pomIm = handles.img2;
                [height width channels] = size(pomIm);
                pomIm = reshape(pomIm,width*height,1);
                pomIm = double(pomIm);
                
                
                %figure;
                %subplot(121); 
                %imshow(reshape(handles.img2,96,96));title('Looking for ...','FontWeight','bold','Fontsize',16,'color','red');
                
                %subplot(122);
                
                poc = poc+1;
                Tstart = tic;
                
                % Predicted person by classificator
                predicted = fisherfaces_predict(faceClassifier, pomIm, 1);
                
                tElapsed = toc(Tstart);
                time_pom = time_pom + tElapsed;
                
                galleryImage2 = read(faceDatabase(predicted),1);
                
                person = string(person);
                personLabel = string(faceDatabase(predicted).Description);  
                                          
                %subplot(122);
                if  strcmp(personLabel,person)
                    found = found+1
                    %imshow(reshape(galleryImage2,96,96));title('Found','FontWeight','bold','Fontsize',16,'color','red');
                else
                    %imshow(reshape(galleryImage2,96,96));title('Matched face','FontWeight','bold','Fontsize',16,'color','red');
                end
                
                % Found the person for the fifth time
                if found == 5
                    axes(handles.axes2);
                    imshow(Im);title('Found');
                    flag = 1;
                    break;
                end
          
                  
            end
            
            % Recognise a person using Eigenface features
            if extract == "eigen2"
                
                % Create a vector out of image
                pomIm = handles.img2;
                [height width channels] = size(pomIm);
                pomIm = reshape(pomIm,width*height,1);
                pomIm = double(pomIm);
                
                
                %figure;
                %subplot(121); 
                %imshow(reshape(handles.img2,96,96));title('Looking for ...','FontWeight','bold','Fontsize',16,'color','red');
                
                %subplot(122);
                
                poc = poc+1;
                Tstart = tic;
                
                % Predicted person by classificator
                predicted = eigenfaces_predict(faceClassifier, pomIm, 3);
                
                tElapsed = toc(Tstart);
                time_pom = time_pom + tElapsed;
                
                galleryImage2 = read(faceDatabase(predicted),1);
                

                person = string(person);
                personLabel = string(faceDatabase(predicted).Description);  
                                
                
                %subplot(122);
                if  strcmp(personLabel,person)
                    found = found+1
                    %imshow(reshape(galleryImage2,96,96));title('Found','FontWeight','bold','Fontsize',16,'color','red');
                else
                    %imshow(reshape(galleryImage2,96,96));title('Matched face','FontWeight','bold','Fontsize',16,'color','red');
                end
                
                % Found the person for the fifth time
                if found == 5
                    axes(handles.axes2);
                    imshow(Im);title('Found');
                    flag = 1;
                    break;
                end
    
            end
            
        end
    catch
    end         
    if flag == 1
        break
    else
       axes(handles.axes2);
       imshow(video_im);title('Person not found');
    end    
end

%Display average time of feature extraction and prediction
average_time_extract = extr/poc;
num2str(average_time_extract)
average_time_predict = time_pom/poc;
num2str(average_time_predict)

end



% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
global extract

extract = "hog";


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3
global extract

extract = "eigen2";


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4
global extract

extract = "fisher";


% --- Executes on button press in radiobutton9.
function radiobutton9_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton9
global extract

extract = "lbp";


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla

%Close all figures
if true
% get( handles.output , 'Tag' ) is the 'Tag' of the GUI
Figures = findobj('Type','Figure','-not','Tag',get(handles.output,'Tag'));
close(Figures)
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global extract faceClassifier faceDatabase databasePath personIndex

% Start training of classifiers

if ~isequal(extract,0)  
disp(extract);


%Path of database on drive
databasePath = '.\P1E_S1_C1';


% Choosing database of faces for training
faceDatabase = imageSet(databasePath,'recursive');


% Calculate the sizes of facedatabase
asize = size(faceDatabase,2); 
bsize = faceDatabase(1).Count;


% Display training database all persons
figure;
for i=1:size(faceDatabase,2)
imageList(i) = faceDatabase(i).ImageLocation(5);
end

montage(imageList);

%{
% Display Montage of First Face
figure;
montage(faceDatabase(1).ImageLocation);
title('Images of Single Face');
            
            
%  Display Query Image and Database Side-Side
personToQuery = 1;
galleryImage = read(faceDatabase(personToQuery),1);
figure;
for i=1:size(faceDatabase,2)
    imageList(i) = faceDatabase(i).ImageLocation(5);
end
subplot(1,2,1);imshow(galleryImage);
subplot(1,2,2);montage(imageList);
diff = zeros(1,9);
%}

Tstart = tic;
% Training classifier with extracted HoG features 
if extract == "hog"

    % Extract and display Histogram of Oriented Gradient Features for single face 
    person = 5;
    [hogFeature, visualization]= extractHOGFeatures(read(faceDatabase(person),3));
    
    
    %figure;
    %subplot(2,1,1);imshow(read(faceDatabase(person),1));title('Input Face');
    %subplot(2,1,2);plot(visualization);title('HoG Feature');
    
    
    [m n] = size(hogFeature);


    % Extract HOG Features for faceDatabase set 
    faceDatabaseFeatures = zeros(size(faceDatabase,2)*faceDatabase(1).Count,n);
    featureCount = 1;
    for i=1:size(faceDatabase,2)
        for j = 1:faceDatabase(i).Count
            faceDatabaseFeatures(featureCount,:) = extractHOGFeatures(read(faceDatabase(i),j));
            faceDatabaseLabel{featureCount} = faceDatabase(i).Description;    
            featureCount = featureCount + 1;
        end
        personIndex{i} = faceDatabase(i).Description;
    end

    % Create 40 class classifier using fitcecoc 
    faceClassifier = fitcecoc(faceDatabaseFeatures,faceDatabaseLabel);
    %faceClassifier = fitcknn(faceDatabaseFeatures,faceDatabaseLabel);
    
    
    disp('Training finished')

% Training classifier with extracted LBP features  
elseif extract == "lbp"

    % Extract Histogram of Oriented Gradient Features for single face 
    person = 5;
    lbpFeature= extractLBPFeatures(read(faceDatabase(person),1));

    [m n] = size(lbpFeature);


    % Extract LBP Features for faceDatabase set 
    faceDatabaseFeatures = zeros(size(faceDatabase,2)*faceDatabase(1).Count,n);
    featureCount = 1;
    for i=1:size(faceDatabase,2)
        for j = 1:faceDatabase(i).Count
            faceDatabaseFeatures(featureCount,:) = extractLBPFeatures(read(faceDatabase(i),j));
            faceDatabaseLabel{featureCount} = faceDatabase(i).Description;    
            featureCount = featureCount + 1;
        end
        personIndex{i} = faceDatabase(i).Description;
    end

    % Create 24 class classifier using fitcecoc 
    faceClassifier = fitcecoc(faceDatabaseFeatures,faceDatabaseLabel);
    %faceClassifier = fitcknn(faceDatabaseFeatures,faceDatabaseLabel);

    disp('Training finished')
    

% Training classifier with extracted fisher features  
elseif extract == "fisher"
    
    % load function files from subfolders aswell
    %addpath (genpath ('.'));
    addpath(genpath('.\m'));
    % load data
    [X y width height names] = read_images('.\P1E_S1_C1');
    
    start = tic;
    % compute a model
    faceClassifier = fisherfaces(X,y);
    
    vel = size(X,2);
  
    %{
    im = imread('tvar.pgm');
    pomIm = im;

    scaleFactor = 96/size(pomIm, 1);
    pomIm = imresize(pomIm,scaleFactor);


    [height1 width1 channels] = size(pomIm);

    pomIm = reshape(pomIm,width1*height1,1);
    pomIm = double(pomIm);
    
    predicted = fisherfaces_predict(faceClassifier, pomIm, 1);
    
    galleryImage2 = read(faceDatabase(predicted),1);
    figure;
    subplot(1,2,1);imshow(im);title('Searching for');
    subplot(1,2,2);montage(galleryImage2);
    title('Predicted');
    %}
    disp('Training finished')
    
    Tend = toc(start)
    fisherextr = Tend/vel

% Training classifier with extracted eigen features  
elseif extract == "eigen2"
    
    % load function files from subfolders aswell
    %addpath (genpath ('.'));
    addpath(genpath('.\m'));
    % load data
    [X y width height names] = read_images('.\P1E_S1_C1');
    
    % compute a model
    start = tic;
    faceClassifier = eigenfaces(X,y);
    vel = size(X,2);
   
    %{
    im = imread('tvar.pgm');
    pomIm = im;

    scaleFactor = 96/size(pomIm, 1);
    pomIm = imresize(pomIm,scaleFactor);


    [height1 width1 channels] = size(pomIm);

    pomIm = reshape(pomIm,width1*height1,1);
    pomIm = double(pomIm);
    
    predicted = eigenfaces_predict(faceClassifier, pomIm, 1);
    
    galleryImage2 = read(faceDatabase(predicted),1);
    figure;
    subplot(1,2,1);imshow(im);title('Searching for');
    subplot(1,2,2);montage(galleryImage2);
    title('Predicted');
    %}
    
    disp('Training finished')    
    
    Tend = toc(start)
    eigenextr = Tend/vel
    
end
tElapsed = toc(Tstart)
end
