/*
 * SECTION etc functions
 */

// from http://css-tricks.com/snippets/javascript/get-url-variables/
function getQueryVariable(variable) {
       var query = window.location.search.substring(1);
       var vars = query.split("&");
       for (var i=0;i<vars.length;i++) {
               var pair = vars[i].split("=");
               if(pair[0] == variable){return pair[1];}
       }
       return(false);
}

function showDetailview () {
	//$('.subPane.overview').css('display', 'none');
	//$('.class_subPane').not('.overlay').css('display', '');
	//$('.overlay').hide();
	for (var c = 0; c < vmr.num_subPane; c++) {
		vmr.overview_subPane[c].css('display', 'none');
	}
}

function showOverview () {
	//$('.class_subPane').not('.overlay').css('display', 'none');
	//$('.overlay').hide();
	//$('.subPane.overview').css('display', '');
	for (var c = 0; c < vmr.num_subPane; c++) {
		vmr.overview_subPane[c].css('display', '');
	}
}

function toggleTimeline (click_field) {
	var elem = $('#pane_timeline');
	if (elem.css('display') === 'none') {
		elem.css('display', '');
		$(click_field).css('background-color', '#123333');
	} else {
		elem.css('display', 'none');
		$(click_field).css('background-color', '#000000');
	}
	console.log("toggled timeline");
}

function toggleTreemap (click_field) {
	var elem = $('#pane_treemap');
	if (elem.css('display') === 'none') {
		elem.css('display', '');
		$(click_field).css('background-color', '#123333');
	} else {
		elem.css('display', 'none');
		$(click_field).css('background-color', '#000000');
	}
	console.log("toggled treemap");
}

function toggleMap (click_field) {
	var elem = $('#pane_map');
	if (elem.css('display') === 'none') {
		elem.css('display', '');
		$(click_field).css('background-color', '#123333');
	} else {
		elem.css('display', 'none');
		$(click_field).css('background-color', '#000000');
	}
	console.log("toggled map");
}

function resetAll () {
	removeAllSelected();
	$('.overlay').css('display', 'none');
	showOverview();
	vmr.panzoom.panzoom("reset");
	vmr.currentScale = 1.0;
}

function toggleZoom () {
	if (vmr.enableZoom === false)
	{
		vmr.enableZoom = true;
	} else {
		vmr.enableZoom = false;
	}
}

/*
 * SECTION search functions
 */

// Adjust the opacity of selected vs. non selected elements
function pruneSelected() {
	if (vmr.selectedNum > 0) {
		$(document).find(".img_wrapper").not(".selected").css("opacity", 0.3);
	} else {
		$(document).find(".img_wrapper").css("opacity", 1.0);
	}
}

// Remove the selected class from all img_wrapper elements
function removeAllSelected() {
	$(document).find(".img_wrapper").removeClass("selected");
	vmr.selectedNum = 0;
	pruneSelected();
}

// Rebuild the subPane of search results, for 's' toggle
function rebuildSearch () {
	console.log("selected count: ", vmr.selectedNum);
	var search_div = $('#search_results_imgs');
	var temp_br_counter = 0;
	search_div.empty();
	console.log('num selected: ', $('.selected').length);
	console.log('num selected pruned: ', $('.selected').find('.class_img').length);
	$('.selected').find('.class_img').clone().each( function (index) {
		//console.log(index + ":" + this);
		search_div.append("<DIV class=\"img_wrapper\" id=\"search_wrapper_" + index + "\"></DIV>");
		search_div.find("#search_wrapper_" + index)
		.append(this);
		if (temp_br_counter >= (Math.sqrt(vmr.selectedNum))) {
			search_div.append("<BR>");
			temp_br_counter = 0;
		} else {
			temp_br_counter++;
		}
	});
	console.log("selected num: ", vmr.selectedNum, ". Sqrt: ", Math.sqrt(vmr.selectedNum));
	if (vmr.selectedNum < 1) {
		hideSearch();
	}
}

function doSearch () {
	$('.overlay').css('display', 'none');
	vmr.panzoom.panzoom("reset");
	vmr.currentScale = 1.0;
	showDetailview();
	showSearch();
}

function showSearch () {
	//$('#search_results').toggleClass('mainPane_above');
	rebuildSearch();
	$('#search_results').css('display', '');
	console.log("show search");
}

function hideSearch () {
	//$('#search_results').toggleClass('mainPane_above');
	$('#search_results').css('display', 'none');
	console.log("toggled search");
}

function selectRand () {
	console.log('imgs/12: ', (vmr.num_imgs / 12));
	for (var i = 0; i < (vmr.num_imgs / 12); i++)
	{
		console.log('selecting rand');
		var randNum = Math.floor(Math.random()*(vmr.num_imgs));
		var randElem = $(".img_wrapper").not('.overview').eq(randNum);
		if (randElem.hasClass('selected'))
		{
			randElem.removeClass("selected");
			vmr.selectedNum--;
		} else {
			randElem.addClass("selected");
			randElem.css("opacity", 1.0);
			vmr.selectedNum++;
		}
	}
	pruneSelected();
}

function removeRand () {
	for (var i = 0; i < 12; i++)
	{
		var randNum = Math.floor(Math.random()*(vmr.selectedNum));
		var randElem = $(".img_wrapper.selected").not('.overview').eq(randNum);
		randElem.removeClass("selected");
		vmr.selectedNum--;
	}
	pruneSelected();
	rebuildSearch();
}

function selectRand_sideToggle () {
	var num = Math.floor(vmr.num_imgs/8);
	var randElem = [];
	for (var i = 0; i < num; i++)
	{
		var randNum = Math.floor(Math.random()*(vmr.num_imgs));
		randElem[i] = $(".img_wrapper").not('.overview').eq(randNum);
		if (randElem[i].hasClass('selected')) {
			randElem[i].removeClass("selected");
			vmr.selectedNum--;
		} else {
			randElem[i].addClass("selected");
			randElem[i].css("opacity", 1.0);
			vmr.selectedNum++;
		}
	}
	pruneSelected();
	return randElem;
}

function removeRand_sideToggle (elems) {
	$(elems).each(function () {
		$(this).removeClass("selected");
		vmr.selectedNum--;
	});
	
	pruneSelected();
	rebuildSearch();
}

function enableSearchButton () {
	$('#draw_submit').prop("disabled", false);
	$('#draw_submit').attr("value", "Valid Path - Search");
}

function disableSearchButton () {
	$('#draw_submit').prop("disabled", true);
	$('#draw_submit').attr("value", "Search Disabled");
}

function toggleSearch () {
	//$('#search_results').toggleClass('mainPane_above');
	rebuildSearch();
	$('#search_results').toggle("fast");
	console.log("toggled search");
}

/*
 * SECTION bin functions
 */

// Performs an animation on a given bin link one a side of mainPane.
function bin_link_flash (bin) {
	var elem = '';
	var img = '';
	switch (bin) {
		case 'top':
			elem = $('#bin_link_top');
			img = '../images/tabs/tab_top';
		break;
		case 'left':
			elem = $('#bin_link_left');
			img = '../images/tabs/tab_left';
		break;
		case 'bottom':
			elem = $('#bin_link_bottom');
			img = '../images/tabs/tab_bottom';
		break;
		case 'right':
			elem = $('#bin_link_right');
			img = '../images/tabs/tab_right';
		break;
	}
	elem.attr('src', img + '_highlight.svg');
	elem.toggle('pulsate', { times: 3 }, 'slow', function () {
		elem.css({
			'opacity': '1.0',
			'display': ''
		});
		elem.attr('src', img + '.svg');
	});
}

// Adds a given set of elemens to a given bin
function addToBin (elements, bin) {
	switch (bin) {
		case 'top':
			elements.addClass('selected_bin_top');
			bin_link_flash('top');
		break;
		case 'left':
			elements.addClass('selected_bin_left');
			bin_link_flash('left');
		break;
		case 'bottom':
			elements.addClass('selected_bin_bottom');
			bin_link_flash('bottom');
		break;
		case 'right':
			elements.addClass('selected_bin_right');
			bin_link_flash('right');
		break;
	}
}

// Reloads selected images into their bin HTML
function rebuildBins (bin) {
	var searchDiv = "null";
	var image = "null";
	var new_image_id = "null";
	var new_image_parent = "null";
	
	//$('.bin_imgs').empty();
	
	switch (bin) {
		case 'top':
			searchDiv = $('#bin_top_imgs');
			searchDiv.empty();
			image = $('.class_subPane').not('.overlay').find('.selected_bin_top').find('.class_img');
			new_image_id = "bin_top_img_";
			new_image_parent = "#bin_top_img_";
		break;
		case 'left':
			searchDiv = $('#bin_left_imgs');
			searchDiv.empty();
			image = $('.class_subPane').not('.overlay').find('.selected_bin_left').find('.class_img');
			new_image_id = "bin_left_img_";
			new_image_parent = "#bin_left_img_";
		break;
		case 'bottom':
			searchDiv = $('#bin_bottom_imgs');
			searchDiv.empty();
			image = $('.class_subPane').not('.overlay').find('.selected_bin_bottom').find('.class_img');
			new_image_id = "bin_bottom_img_";
			new_image_parent = "#bin_bottom_img_";
		break;
		case 'right':
			searchDiv = $('#bin_right_imgs');
			searchDiv.empty();
			image = $('.class_subPane').not('.overlay').find('.selected_bin_right').find('.class_img');
			new_image_id = "bin_right_img_";
			new_image_parent = "#bin_right_img_";
		break;
	}
	
	image.clone().each( function (index) {
		searchDiv.append('<DIV class="img_wrapper" id="' + new_image_id + index + "\"></DIV>");
		searchDiv.find(new_image_parent + index).append(this);
		if ((bin === 'left' || bin === 'right') && ((index+1)%2 === 0)) {
			searchDiv.append('<BR>');
		}
	});
}

function toggleBinTop () {
	rebuildBins('top');
	//$('#mainPane_overlay').toggleClass('mainPane_above');
	$('#bin_top').toggle("slow");
	console.log("toggled bin_top");
}

function toggleBinLeft () {
	rebuildBins('left');
	//$('#mainPane_overlay').toggleClass('mainPane_above');
	$('#bin_left').toggle("slow");
	console.log("toggled bin_left");
}

function toggleBinBottom () {
	rebuildBins('bottom');
	//$('#mainPane_overlay').toggleClass('mainPane_above');
	$('#bin_bottom').toggle("slow");
	console.log("toggled bin_bottom");
}

function toggleBinRight () {
	rebuildBins('right');
	//$('#mainPane_overlay').toggleClass('mainPane_above');
	$('#bin_right').toggle("slow");
	console.log("toggled bin_right");
}

/*
 * SECTION drawing functions
 */

function toggleDrawBegin () {
	$('#draw_menu_begin').toggle();
	console.log('toggled drawBegin');
}

function showDrawBegin () {
	$('#draw_menu_begin').css('display', '');
	console.log('show drawBegin');
}

function hideDrawBegin () {
	$('#draw_menu_begin').css('display', 'none');
	console.log('hide drawBegin');
}

function showDrawMenu () {
	$('#draw_menu').css('display', '');
	console.log('show draw_menu');
}

function hideDrawMenu () {
	$('#draw_menu').css('display', 'none');
	disableSearchButton();
	console.log('hide draw_menu');
}

function toggleDraw () {
	if (vmr.draw === false)
	{
		console.log(vmr.sketch_width);
		console.log(vmr.sketch_height);
		
		vmr.draw = true;
		
		$('#mainPane').append('<DIV id="mainPane_overlay"></DIV>');
		
		vmr.sketchpad = Raphael.sketchpad("mainPane_overlay", {
			width: vmr.sketch_width,
			height: vmr.sketch_height,
			editing: true
		});
		
		vmr.panzoom.panzoom("option", {
			disablePan: true,
			disableZoom: true
		});
		
		$('#mainPane_overlay').css("z-index", 700);
		
		hideDrawBegin();
		showDrawMenu();
		
		vmr.sketchpad.change(function() {
			console.log("drawing changed.");
			var stroke_local = vmr.sketchpad.strokes();
			
			if (vmr.draw === false) {
				return;
			} else if (stroke_local.length < 1) {
				console.log('empty stroke_local, exiting path detection');
				return;
			}
			
			//console.log(stroke_local);
			//var item = stroke_local[0]["path"];
			//var len = item.length;
			//var svgBits = item.split("L");
			//var count = item.split("L").length;
			
			var item = stroke_local[0]["path"];
			var count = item.length;
			
			var sub1 = "";
			var sub2 = "";
			
			for (var c = 1; c < (count/2)-1; c++) {
				if (sub1 === "") {
					sub1 = sub1 + "M" + item[c][1] + "," + item[c][2];
				} else {
					sub1 = sub1 + "L" + item[c][1] + "," + item[c][2];
				}
			}
			
			for (var c = Math.floor((count/2)+1); c < count; c++) {
				if (sub2 === "") {
					sub2 = sub2 + "M" + item[c][1] + "," + item[c][2];
				} else {
					sub2 = sub2 + "L" + item[c][1] + "," + item[c][2];
				}
			}
			
			/*
			console.log(count);
			console.log(item);
			console.log(sub1);
			console.log(sub2);
			*/
			
			var result = Raphael.pathIntersection(sub1, sub2);
			
			console.log(result);
			
			if (result.length > 0 && result.length < 4) {
				enableSearchButton();
			}
		});
		
	} else {
		$('#mainPane_overlay').remove();
		vmr.draw = false;
		vmr.panzoom.panzoom("option", {
			disablePan: false,
			disableZoom: false
		});
		hideDrawMenu();
	}
}

function toggleDrawMenu () {
	$('#draw_menu').toggle();
	disableSearchButton();
	console.log('toggled draw_menu');
}
