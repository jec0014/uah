/*
		 * Instantiate the main object
		 */
		var vmr = new Object();
		vmr.mainPane = $('#id_vmr.mainPane');
		vmr.element = vmr.mainPane.find('#panzoom');
		vmr.currentScale = 1.0;
		vmr.enableZoom = true;
		vmr.enablePan = true;
		vmr.minScale = 1.0;
		vmr.maxScale = 15.0;
		vmr.animate = true;
		vmr.transitionTime = 300;
		vmr.increment = (vmr.currentScale);
		vmr.canClick = true;
		vmr.selectedNum = 0;
		vmr.draw = false;
		vmr.sketch_width = $('#vmr.mainPane_overlay').width();
		vmr.sketch_height = $('#vmr.mainPane_overlay').height();
		vmr.sketchpad = "NULL";
		vmr.overview_clones = [];
		vmr.overview_temp = 'NULL';
		
		/*
		 * Debugging output goes here for now...
		 */
		 
		//console.log(vmr.sketchpad.width);
		//console.log(vmr.sketchpad.height);
		
		/*
		 * Function declarations
		 */
		 
		// Adjust the opacity of selected vs. non selected elements
		function pruneSelected() {
			if (vmr.selectedNum > 0) {
				$(document).find(".img_wrapper").not(".selected").css("opacity", 0.3);
			} else {
				$(document).find(".img_wrapper").not(".selected").css("opacity", 1.0);
			}
		}	
		
		// Remove the selected class from all img_wrapper elements
		function removeAllSelected() {
			$(document).find(".img_wrapper").removeClass("selected");
			vmr.selectedNum = 0;
			pruneSelected();
		}
		
		// Rebuild the subPane of search results, for 's' toggle
		function rebuildSearchResults () {
			var search_div = $('#search_results_imgs');
			var temp_br_counter = 0;
			search_div.empty();
			$('.selected').clone().each( function (index) {
				//console.log(index + ":" + this);
				search_div.append("<DIV class=\"img_wrapper\" id=\"search_wrapper_" + index + "\"></DIV>");
				search_div.find("#search_wrapper_" + index)
				.append(this);
				/*
				if (temp_br_counter >= 2) {
					this.append("<BR>");
					temp_br_counter = 0;
				} else {
					temp_br_counter++;
				}
				*/
			});
		}
		
		function toggleOverview (element) {
			element.addClass('invisible');
			if (element.hasClass('subPane')) {
				
			}
		}
		
		/*
		 * Hotkey definitions
		 */	
		 	
		// On 'z', toggle the zoom flag
		$(document).bind('keypress', 'z', function() {
			if (vmr.enableZoom === false)
			{
				vmr.enableZoom = true;
			} else {
				vmr.enableZoom = false;
			}
		});
		
		// On 'l', reset all of panzoom and raphael
		$(document).bind('keydown', 'l', function() {
			$("#panzoom").panzoom("reset");
			removeAllSelected();
		});
		
		// On 'a', find 10 random images and give them the .selected class
		$(document).bind('keypress', 'a', function() {
			for (var i = 0; i < 10; i++)
			{
				var randNum = Math.floor(Math.random()*600);
				var randElem = $(".img_wrapper").eq(randNum);
				randElem.addClass("selected");
				randElem.css("opacity", 1.0);
				vmr.selectedNum++;
			}
			pruneSelected();
		});
		
		// On 'i', go to the detail view
		$(document).bind('keypress', 'i', function() {
			$('.subPane.overview').addClass('invisible');
			$('.class_subPane').removeClass('invisible');
			$('#search_results').addClass('invisible');
		});
		
		// On 'o', go to the overview
		$(document).bind('keypress', 'o', function() {
			$('.class_subPane').addClass('invisible');
			$('#search_results').addClass('invisible');
			$('.subPane.overview').removeClass('invisible');
		});
		
		// On 'p', go to the search result view
		$(document).bind('keypress', 'p', function() {
			$('.class_subPane').addClass('invisible');
			$('.subPane.overview').addClass('invisible');
			$('#search_results').removeClass('invisible');
		});
		
		// On 'd', toggle the ability to draw on vmr.mainPane
		$(document).bind('keypress', 'd', function() {
			if (vmr.draw === false)
			{
				vmr.draw = true;
				$('#id_vmr.mainPane').append('<DIV id="vmr.mainPane_overlay"></DIV>');
				vmr.sketchpad = Raphael.sketchpad("vmr.mainPane_overlay", {
					width: vmr.sketch_width,
					height: vmr.sketch_height,
					editing: true
				});
				vmr.element.panzoom("option", {
					disablePan: true,
					disableZoom: true
				});
				$('#vmr.mainPane_overlay').css("z-index", 500);
			
			} else {
				
				$('#vmr.mainPane_overlay').remove();
				vmr.draw = false;
				vmr.element.panzoom("option", {
					disablePan: false,
					disableZoom: false
				});
				$('#vmr.mainPane_overlay').css("z-index", 300);
			}
		});
		
		// On 'c', clear the sketchpad without destroying it
		$(document).bind('keypress', 'c', function() {
			vmr.sketchpad.clear();
		});
		
		// On 'q' test something
		$(document).bind('keypress', 'q', function() {
			rebuildSearchResults();
		});
		
		
		vmr.element.panzoom({
			contain: 'invert'
			});
			
		$('#id_vmr.mainPane').on('mousewheel.focal', function(e) {
			if (vmr.enableZoom === true)
			{
				e.preventDefault();
				var delta = e.delta || e.originalEvent.wheelDelta;
				var zoomOut = delta ? delta < 0 : e.originalEvent.deltaY > 0;
				vmr.element.panzoom('zoom', zoomOut, {
					increment: vmr.increment,
					transition: true,
					duration: vmr.transitionTime,
					focal: e,
					minScale: vmr.minScale,
					maxScale: vmr.maxScale
				});
			}
			
			/*
			console.log("VMR: ");
			console.log(vmr.currentScale);
			console.log(vmr.increment);
			*/
			
		});
		
		
		vmr.element.panzoom().on('panzoompan', function() {
				vmr.canClick = false;
			});
		
		
		$(document).find(".img_wrapper").click(function() {
			if (vmr.draw === false)
			{
				if (vmr.canClick === false)
				{
					vmr.canClick = true;
				} 
				else if ($(this).hasClass("selected") == false) 
				{
					var imgBorder = $(this).css("border");
					$(this).addClass("selected");
					$(this).css("opacity", 1.0);
					vmr.selectedNum++;
				} else {
					$(this).removeClass("selected");
					vmr.selectedNum--;
				}
			}
		pruneSelected();
		});
		
		
		$(document).find(".class_img").dblclick(function(e) {
			vmr.element.panzoom('zoom', 25.0, {
				animate: true,
				focal: e
			});
		});
		
		/*
		$(".subPaneFooter").dblclick(function(e) {
			/*
			var localElement = $(this).parent();
			console.log(localElement);
			console.log(localElement.hasClass('overview_toggle_0'));
			$(localElement).addClass('invisible');
			var newElem = $(localElement).clone();
			$(newElem).replaceWith($('.subPane.overview_toggle_0'));
			$(newElem).removeClass('invisible');
			/
			//if (newElem.hasClass('overview_toggle_0')) {
			//	$(newElem).toggleClass('invisible');
			//}
			
			var localElement = $(this).parent();
			console.log(localElement);
			console.log(vmr.overview_clones);
			
			vmr.overview_temp = $(localElement).clone();
			$(localElement).replaceWith(vmr.overview_clones[0]);
			vmr.overview_clones[0] = vmr.overview_temp;
			
		});
		*/
		
		$(document).find(".img_overlay").hover(function() {
		$(this).animate({
			opacity: 0.7
			}, vmr.transitionTime);
		}, function() {
			$(this).animate({
				opacity: 0.3
				}, vmr.transitionTime);
		});
		
		$(document).find(".img_wrapper").hover(function() {
			$(this).find(".img_overlay").animate({
				opacity: 0.3
				}, vmr.transitionTime);
		}, function() {
			$(this).find(".img_overlay").animate({
				opacity: 0.0
				}, vmr.transitionTime);
		});
		
		$(document).find(".img_overlay").click(function(e) {
			$(this).parent().find(".img_wrapper").addClass("selected");
		});



var subPaneWidth = [];
	var subPaneHeight = [];
	var img_num_across = [10,7,24,21,20];
	var img_num_tall = [1,1,1,1,1]; // Generated later
	
	// Generate detail subPanes
	for (var c1 = 0; c1 < 2 /*db_img.length*/; c1++)
	{
		// Create subPane
		$('body').find("#panzoom").append("<DIV class=\"class_subPane overview_toggle_" + c1 + "\" id=\"subPane_" + db_img[c1][0] + "\"></DIV>");
		
		for (var c2 = 1; c2 < db_img[c1].length /*&& c2 < 10*/; c2++)
		{
			// Insert image wrapper (has border)
			$('body').find("#subPane_" + db_img[c1][0]).append("<DIV class=\"img_wrapper\" id=\"wrapper" + c2 + "\"></DIV>");
			
			// Insert image
			$('body').find("#subPane_" + db_img[c1][0]).find("#wrapper" + c2)
			.append("<IMG class=\"class_img\" src=\"../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + "\" />");
			
			/*
			$('body').find("." + db_img[c1][0]).find("#wrapper" + c2)
			.append("<IMG class=\"img_overlay\" src=\"../images/img_overlay.png\" />");
			*/
			
			// SMAP laptop screen width: 41 imgs
			switch (c1) {
				case 0: // Buildings
					if (c2%(img_num_across[c1]) === 0) {
						$("#subPane_" + db_img[c1][0]).append("<BR>");
						img_num_tall[c1]++;
					}
				break;
				case 1: // Misc
					if (c2%(img_num_across[c1]) === 0) {
						$("#subPane_" + db_img[c1][0]).append("<BR>");
						img_num_tall[c1]++;
					}
				break;
				case 2: // People
					if (c2%(img_num_across[c1]) === 0) {
						$("#subPane_" + db_img[c1][0]).append("<BR>");
						img_num_tall[c1]++;
					}
				break;
				case 3: // Vehicles
					if (c2%(img_num_across[c1]) === 0) {
						$("#subPane_" + db_img[c1][0]).append("<BR>");
						img_num_tall[c1]++;
					}
				break;
				case 4: // Weapons
					if (c2%(img_num_across[c1]) === 0) {
						$("#subPane_" + db_img[c1][0]).append("<BR>");
						img_num_tall[c1]++;
					}
				break;
				default: // Default action
					console.log("error in image <br> generation switch");
				break;
			} /* end switch */
		
		} /* end for c2 */
		
		// Attach subPane footer
		$('body').find("#subPane_" + db_img[c1][0]).append("<DIV class=\"subPaneFooter\"></DIV>");
		
		// Insert subPane background cluster icon
		$('body').find("#subPane_" + db_img[c1][0]).find(".subPaneFooter")
		.css("background-image", "url(../images/cluster_icon/cluster_detail_" + c1 + ".svg)");
		
		// Insert image count
		$('body').find("#subPane_" + db_img[c1][0]).find(".subPaneFooter")
		.append('<div class="text">' + (c2-1) + '</div>');
		
		subPaneWidth[c1] = $("#subPane_" + db_img[c1][0]).width();
		subPaneHeight[c1] = $("#subPane_" + db_img[c1][0]).height();
		
	} /* end for c1 */
	
	// Generate Overview subPanes
	for (var c1 = 0; c1 < db_img.length; c1++)
	{
		// Create subPane
		$('body').find("#panzoom")
		.append("<DIV class=\"subPane overview_toggle_" + c1 + " overview invisible\" id=\"subPane_overview_" 
			+ db_img[c1][0] + "\"></DIV>");
		
		vmr.overview_clones[c1] = $('#subPane_overview_' + db_img[c1][0]);
		
		$('#subPane_overview_' + db_img[c1][0]).width(subPaneWidth[c1] + 'px');
		$('#subPane_overview_' + db_img[c1][0]).height(subPaneHeight[c1] + 'px');
		
		$('#subPane_overview_' + db_img[c1][0])
			.append('<DIV id="subPane_imgFrame_' + db_img[c1][0] + '" class="imgFrame overview" style="height: ' 
			+ (subPaneHeight[c1]-28) + 'px; width: ' + (subPaneWidth[c1]) 
			+ 'px; text-align: center; overflow: hidden; display: inline-block;"></DIV>');
		
		for (var c2 = 1; c2 < db_img[c1].length && c2 < 6; c2++)
		{
			
			/*
			// Insert image wrapper (has border)
			$('body').find('#subPane_imgFrame_' + db_img[c1][0])
				.append("<DIV class=\"img_wrapper overview item\" id=\"wrapper_overview_" + c2 + "\"></DIV>");
			
			// Insert image
			$('body').find('#subPane_imgFrame_' + db_img[c1][0]).find("#wrapper_overview_" + c2)
				.append("<IMG class=\"class_img overview\" src=\"../images/database/"
				+ db_img[c1][0] + "/" + db_img[c1][c2] + "\" />");
			*/
			
			// Insert image wrapper (has border)
			$('body').find('#subPane_imgFrame_' + db_img[c1][0])
				.append("<DIV class=\"img_wrapper overview item\" id=\"wrapper_overview_" + c2 	+ '"' + ' ></DIV>');
				//+ ' style="width: ' + Math.floor((subPaneWidth[c1]-1)/(c2+2)) + 'px; height: ' + Math.floor((subPaneHeight[c1]-23)/(c2+2)) 
				//+ "px; background: url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ') no-repeat center;"'
				//+ " ></DIV>");
			
			var imgFrameWidth = $('#subPane_imgFrame_' + db_img[c1][0]).width();
			var imgFrameHeight = $('#subPane_imgFrame_' + db_img[c1][0]).height();
			
			var wrapperWidth = [[],[],[],[],[]];
			var wrapperHeight = [[],[],[],[],[]];
			
			wrapperWidth[c1][c2] = (Math.floor((imgFrameWidth)/2)-9) + 'px';
			wrapperHeight[c1][c2] = (Math.floor((imgFrameHeight)/2)) + 'px';
			
			if (c2 > 2) {
				wrapperWidth[c1][c2] = (Math.floor((imgFrameWidth)/3)-12) + 'px';
				wrapperHeight[c1][c2] = (Math.floor((imgFrameHeight)/3)) + 'px';
			}
				
			//console.log(wrapperWidth[c1][c2] + " , " + wrapperHeight[c1][c2]);
			
			var switchElem = $('#subPane_imgFrame_' + db_img[c1][0]);
			
			switch (c2) {
				
			case 1:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					"background": "url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 2:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					"background": "url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 3:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					"background": "url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 4:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					"background": "url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			case 5:
				switchElem.find('#wrapper_overview_' + c2).css({
					"width": wrapperWidth[c1][c2],
					"height": wrapperHeight[c1][c2],
					"background": "url(../images/database/" + db_img[c1][0] + "/" + db_img[c1][c2] + ") no-repeat center",
					"background-size": wrapperWidth[c1][c2] + "px " +  wrapperHeight[c1][c2] + "px"
				});
			break;
			
			}
			
			/*
			$('body').find("." + db_img[c1][0]).find("#wrapper" + c2)
			.append("<IMG class=\"img_overlay\" src=\"../images/img_overlay.png\" />");
			*/
		} /* end for c2 */
		
		// Attach subPane footer
		$('body').find("#subPane_overview_" + db_img[c1][0]).append("<DIV class=\"subPaneFooter overview\"></DIV>");
		
		// Insert subPane background cluster icon
		$('body').find("#subPane_overview_" + db_img[c1][0])
			.find(".subPaneFooter.overview")
			.css("background-image", "url(../images/cluster_icon/cluster_detail_" + c1 + ".svg)");
		
		// Insert image count
		$('body').find("#subPane_overview_" + db_img[c1][0])
			.find(".subPaneFooter.overview")
			.append('<div class="text overview">' + c2 + '</div>');
		
		/*
		var wall = new freewall('#subPane_imgFrame_' + db_img[c1][0]);
		wall.reset({
			selector: '.item',
			animate: false,
			onResize: function() {
				wall.fitWidth();
			}
		});
		wall.fitZone((subPaneWidth[c1]-1), (subPaneHeight[c1]-23));
		*/
		
	} /* end for c1 */
