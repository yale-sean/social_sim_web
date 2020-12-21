Qualtrics.SurveyEngine.addOnload(function()
{
});

Qualtrics.SurveyEngine.addOnReady(function()
{
  var link = false;
  var intro = false;
  var url = "https://${e://Field/hostname}?scene=${lm://Field/1}&avatar=${lm://Field/2}&person_position=${lm://Field/3}&robot_position=${lm://Field/4}&avatar_seed=${lm://Field/5}&t1=${lm://Field/6}&t2=${lm://Field/7}&id=${e://Field/id}";
  if (intro) {
    url = "https://${e://Field/hostname}?scene=lab&avatar=f1&person_position=SpawnLocation_Lab7&robot_position=SpawnLocation_Lab6&avatar_seed=1&t1=red-turtle&t2=black-turtle&intro=1&id=${e://Field/id}";
  }


  var parent = document.getElementById('insert-link');

  if (link) {
    var a = document.createElement('a');
    a.appendChild(document. createTextNode("Click here to open the simulator."));
    a.setAttribute('id', 'game-link');
    a.setAttribute('target', 'simulator');
    a.setAttribute('href',url);
    parent.parentNode.insertBefore(a,parent);
  } else {
    var ifrm = document.createElement('iframe');
    ifrm.setAttribute('allowfullscreen', '');
    ifrm.setAttribute('scrolling', 'no');
    ifrm.setAttribute('style', 'width:1245px;height:930px;border:1px solid #666;overflow:hidden;');
    ifrm.setAttribute('src', url);
    parent.parentNode.insertBefore(ifrm,parent);
  }
});

Qualtrics.SurveyEngine.addOnUnload(function()
{
/*Place your JavaScript here to run when the page is unloaded*/

});

