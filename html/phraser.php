<?php

/*	
	This program parses inputs from a client (thissfile.php?M1=100&M2=...)
	and writes them to a file (/dev/shm/input.txt) in RAM.
	
	The file was/will be created and given the approprate permissions 
	in the Python program's setup.
	
	The python program reading the file searches the input text string 
	(in /dev/shm/input.txt) for a variable header (e.x. 'M1=' or 'CAQ=') 
	and an end string (i.e. ' - '), and assumes the value it needs, as 
	defined by the header, is between the two.
	
	Visit thestuffwebuild.com for more details.
*/

$microTime = round(microtime(true) * 1000);

//Read inputs and assign to variables.
$Param1 = $_GET['M1']; //speed of left motor 				-100 to 100
$Param2 = $_GET['M2']; //speed of right motor 				-100 to 100
$Param3 = $_GET['CAY']; //cam y pixels						0+
$Param4 = $_GET['CAF']; //cam fps							0+
$Param5 = $_GET['CAQ']; //cam quality						0+
$Param6 = $_GET['LED']; //main led state					1 or 0
$Param7 = $_GET['TTS']; //text to speech phrase 			"arb text"
$Param8 = $_GET['CT']; //client time 						0+
$Param9 = $_GET['ID']; //mode change 						0+
$Param10 = $_GET['SRV']; //servo pos command 							0 to 180
$Param99 = $_GET['GIMMIE']; //if 1 respond with full xml 	1 or 0

//Text to speech housekeeping. Strip unwanted chars and restrict length.
$Param7 = preg_replace("/[^a-zA-Z.,?!0-9\s]/", "", $Param7);
$Param7 = mb_strimwidth($Param7, 0, 140, "");

//Test if inputs are valid and append them to the string $txt. 
$txt = '';
if (is_numeric($Param1)) $txt .= "M1=$Param1 - ";
if (is_numeric($Param2)) $txt .= "M2=$Param2 - ";
if (is_numeric($Param3)) $txt .= "CAY=$Param3 - ";
if (is_numeric($Param4)) $txt .= "CAF=$Param4 - ";
if (is_numeric($Param5)) $txt .= "CAQ=$Param5 - "; 
if (is_numeric($Param6)) $txt .= "LED=$Param6 - "; 
if($Param7 != "") $txt .= "TTS=$Param7 - "; 
if (is_numeric($Param8)) $txt .= "CT=$Param8 - "; 
if (is_numeric($Param9)) $txt .= "ID=$Param9 - "; 
if (is_numeric($Param10)) $txt .= "SRV=$Param10 - ";

//nullify $Param99 if not a number
if (!is_numeric($Param99)) $Param99 = "";

//Open, write to, and close file
$myfile = fopen("/dev/shm/input.txt", "w") or die("Unable to open file!"); //trying
fwrite($myfile, $txt);
fclose($myfile);

//Begin xml data
header('Content-type: text/xml');
echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
echo "<data>";

//send the server time
echo "<ST>";
echo $microTime;
echo "</ST>";

//Output full data file if requested
if ($Param99 == 1) { //should this be "1", not 1?
	$file = file_get_contents('/dev/shm/output.txt');
	echo $file;
}

//Close xml data
echo "</data>";

?>
