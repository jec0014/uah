//Copy everything here and replace my old code with it
var AdvFlag = 0;
var PeopleFlag =0;
document.onmouseup = mouseUp;

function init()
{

	var banWrapper = document.getElementById("Banners");
	var topOff = document.getElementById("TopBanner").offsetHeight;
	var sideBan = document.getElementById("SideBanner");

	sideBan.style.marginTop = Number(topOff);
	
/*
	var sideOff = sideBan.offsetWidth;
	var play = document.getElementById("playArea");
	var playWidth = play.offsetWidth;
	var playHeight = play.offsetHeight;

	play.style.width = Number(playWidth)-Number(sideOff);
	//alert("PLAY:"+ playHeight+"TOP OFF:"+topOff);
	play.style.height = Number(playHeight)-Number(topOff);
	play.style.marginLeft = Number(sideOff);
	play.style.marginTop = Number(topOff);
*/

}

function openTree()
{
		//alert(leftFlag);
		if(PeopleFlag == 1)
		{
			PeopleFlag =0;
			AdvFlag = 0;
		}
		else
		{
			PeopleFlag = 1;
		}
		var root = document.getElementById("PersonSearch");
		var leaf1 =document.getElementById("Tier1-1");

		if(root.className == "root open")
		{
			root.className = "root";
			leaf1.className = "branch";
		}
		else
		{
			root.className+=(" open");
			leaf1.className+=(" open");

		}
}

function openTree2()
{
	//This is done to prevent the toggle from turning the search off if the user is going from 
	//Basci search to Advanced search
	PeopleFlag = 1;
	AdvFlag = 1;
	var leaf2 =document.getElementById("Tier1-2");
	var leaf1 =document.getElementById("Tier1-1");

	if(leaf2.className == "branch open")
	{
		leaf1.className = "branch open";
		leaf2.className = "branch";
	}
	else
	{
		leaf1.className+=(" select");
		leaf2.className+=(" open");
	}
}

function openTree3()
{
	AdvFlag = 1;
	var leaf3 =document.getElementById("Tier1-3");
	var leaf2 =document.getElementById("Tier1-2");

	if(leaf3.className == "branch open")
	{
		leaf3.className = "branch";
		leaf2.className = "branch open";
	}
	else
	{
		leaf3.className+=(" open");
		leaf2.className+=(" select");
	}
}

function openTree4()
{
	AdvFlag = 1;
	var leaf4 =document.getElementById("Tier1-4");
	var leaf3 =document.getElementById("Tier1-3");

	if(leaf4.className == "branch open")
	{
		leaf4.className = "branch";
		leaf3.className = "branch open";
	}
	else
	{
		leaf4.className+=(" open");
		leaf3.className+=(" select");
	}
}

function openTree5()
{
	AdvFlag = 1;
	var leaf5 =document.getElementById("Tier1-5");
	var leaf4 =document.getElementById("Tier1-4");

	if(leaf5.className == "branch open")
	{
		leaf5.className = "branch";
		leaf4.className = "branch open";
	}
	else
	{
		leaf5.className+=(" open");
		leaf4.className+=(" select");
	}
}
function openTree6()
{
	AdvFlag = 1;
	var leaf6 =document.getElementById("Tier1-6");
	var leaf5 =document.getElementById("Tier1-5");

	if(leaf6.className == "branch open")
	{
		leaf6.className = "branch";
		leaf5.className = "branch open";
	}
	else
	{
		leaf6.className+=(" open");
		leaf5.className+=(" select");
	}
}
function FinalBranch()
{
	AdvFlag = 1;
	var leaf6 =document.getElementById("Tier1-6");
	
	/*if(leaf6.className == "branch open")
	{
		leaf6.className = "branch";
	}
	else*/ if(leaf6.className == "branch open select")
	{
		leaf6.className = "branch open";
	}
	else
	{
		leaf6.className+=(" select");
	}
}

function EndTree()
{

	var root = document.getElementById("PersonSearch");
	var leaf1 = document.getElementById("Tier1-1");
	var leaf2 = document.getElementById("Tier1-2");
	var leaf3 = document.getElementById("Tier1-3");
	var leaf4 = document.getElementById("Tier1-4");
	var leaf5 = document.getElementById("Tier1-5");
	var leaf6 = document.getElementById("Tier1-6");

	if(PeopleFlag == 1)
	{
		//root.className = "root select";
		root.className = "root";
		//alert("Root Set");
		//alert(AdvFlag);
		if(AdvFlag == 1)
		{
			//root.className = "root filtered";
			root.className = "root";
			AdvFlag = 0;
		}
		else
		{
			AdvFlag = 0;
		}
	}
	else
	{
					root.className = "root";
	}
	/*else
	{
		AdvFlag = 0;
		root.className = "root";
	}*/
	leaf1.className = "branch";
	//alert("Tier 1-1 closed");
	leaf2.className = "branch";
	//alert("Tier 1-2 closed");
	leaf3.className = "branch";
	//alert("Tier 1-3 closed");
	leaf4.className = "branch";
	//alert("Tier 1-4 closed");
	leaf5.className = "branch";
	//alert("Tier 1-5 closed");
	leaf6.className = "branch";
	//alert("Tier 1-6 closed");
	
	selectRand();
	doSearch();
	
}

function mouseUp(e)
{
	if ($('#PersonSearch').hasClass('open'))
	{
		EndTree();
		//alert(PeopleFlag);
	}
}

function SideToggle(x)
{
	var temp = 404;
	switch(x)
	{
		case 0: 	temp = document.getElementById("PersonSearch");
					break;
		case 1: 	temp = document.getElementById("VehicleSearch");
					break;
		case 2: 	temp = document.getElementById("WeaponSearch");
					break;
		case 3: 	temp = document.getElementById("BuildingSearch");
					break;
		case 4: 	temp = document.getElementById("MeetingSearch");
					break;
		case 5: 	temp = document.getElementById("GroupSearch");
					break;
		case 6: 	temp = document.getElementById("CustomSearch");
					break;
	}
	if(temp != 404)
	{
		if(temp.className == "root")
		{
			temp.className += " select";
		}
		else
		{
			temp.className = "root";
		}
	}
	//alert("Side toggle called with "+x);
}

function SideToggle1()
{
	 SideToggle(1);
	 if ($('#VehicleSearch').hasClass('select')) {
		vmr.vehicles_search = selectRand_sideToggle();
		doSearch();
		console.log("vehicles search: ", vmr.vehicles_search);
	} else {
		removeRand_sideToggle(vmr.vehicles_search);
		vmr.vehicles_search = '';
		//hideSearch();
	}
}
function SideToggle2()
{
	 SideToggle(2);
	 if ($('#WeaponSearch').hasClass('select')) {
		vmr.weapons_search = selectRand_sideToggle();
		doSearch();
		console.log("vehicles search: ", vmr.weapons_search);
	} else {
		removeRand_sideToggle(vmr.weapons_search);
		vmr.weapons_search = '';
		//hideSearch();
	}
}
function SideToggle3()
{
	 SideToggle(3);
	 if ($('#BuildingSearch').hasClass('select')) {
		selectRand();
		doSearch();
	} else {
		removeRand();
	}
}
function SideToggle4()
{
	 SideToggle(4);
	 if ($('#MeetingSearch').hasClass('select')) {
		selectRand();
		doSearch();
	} else {
		removeRand();
	}
}
function SideToggle5()
{
	 SideToggle(5);
	 if ($('#GroupSearch').hasClass('select')) {
		selectRand();
		doSearch();
	} else {
		removeRand();
	}
}
function SideToggle6()
{
	 SideToggle(6);
	 if ($('#CustomSearch').hasClass('select')) {
		selectRand();
		doSearch();
	} else {
		removeRand();
	}
}

/*
function SideToggle_people()
{
	 SideToggle(0);
	 if ($('#PersonSearch').hasClass('select')) {
		selectRand();
		doSearch();
	} else {
		removeRand();
	}
}
*/

//Safeguard to attempt to prevent errors when doubleclicking
function derp()
{
	EndTree();
}
