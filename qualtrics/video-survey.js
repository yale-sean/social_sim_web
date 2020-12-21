Qualtrics.SurveyEngine.addOnload(function()
{
});

Qualtrics.SurveyEngine.addOnReady(function()
{

  var url = "https://survey.interactive-machines.com/rendered?id=${e://Field/id}&animal=${lm://Field/8}"
  var parent = document.getElementById('insert-link');

  var ifrm = document.createElement('iframe');
  ifrm.setAttribute('allowfullscreen', '');
  ifrm.setAttribute('scrolling', 'no');
  ifrm.setAttribute('style', 'width:670px;height:510px;border:none;overflow:hidden;');
  ifrm.setAttribute('src', url);
  parent.parentNode.insertBefore(ifrm,parent);
});

Qualtrics.SurveyEngine.addOnUnload(function()
{
/*Place your JavaScript here to run when the page is unloaded*/

});


