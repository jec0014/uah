// On 'z', toggle the zoom flag
$(document).bind('keypress', 'z', function() {
	toggleZoom();
});

// On 'l', reset all of panzoom and raphael
$(document).bind('keydown', 'l', function() {
	resetAll();
});

// On 'a', find 10 random images and give them the .selected class
$(document).bind('keypress', 'a', function() {
	selectRand();
});

// On 'i', go to the detail view
$(document).bind('keypress', 'i', function() {
	showDetailview();
});

// On 'o', go to the overview
$(document).bind('keypress', 'o', function() {
	showOverview();
});

// On 'p', go to the search result view
$(document).bind('keypress', 'p', function() {
	toggleSearch();
});

// On 'd', toggle the ability to draw on vmr.mainPane
$(document).bind('keypress', 'd', function() {
	toggleDraw();
});

// On 'c', clear the sketchpad without destroying it
$(document).bind('keypress', 'c', function() {
	vmr.sketchpad.clear();
});

// On 'q' test something
$(document).bind('keypress', 'q', function() {
	rebuildSearch();
});

// On 'b' toggle the treemap
$(document).bind('keypress', 'b', function() {
	toggleTreemap($('#Files'));
});

// On 'n' toggle the timeline
$(document).bind('keypress', 'n', function() {
	toggleTimeline($('#MediaTimeline'));
});

// On 'm' toggle the map
$(document).bind('keypress', 'm', function() {
	toggleMap($('#MediaLocation'));
});

// On 'g' toggle the bin_top
$(document).bind('keypress', 'g', function() {
	toggleBinTop();
});

// On 'h' toggle the bin_left
$(document).bind('keypress', 'h', function() {
	toggleBinLeft();
});

// On 'j' toggle the search
$(document).bind('keypress', 'j', function() {
	toggleBinBottom();
});

// On 'k' toggle the search
$(document).bind('keypress', 'k', function() {
	toggleBinRight();
});
